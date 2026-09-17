/*
 * qd_image.c - read-only Sharp Quick Disk .qd decoder
 *
 * Physical HxC/FlashFloppy images store an LSB-first MFM bit-cell stream.
 * There is no guaranteed byte phase, so all sixteen possible data-cell
 * phases are decoded.  Only Sharp frames preceded by break/sync bytes and
 * passing the on-disk CRC are accepted.
 */

#include "qd_image.h"

#include <stdlib.h>
#include <string.h>

#define QD_MAX_FILES              50u
#define QD_MAX_BLOCKS             ( QD_MAX_FILES * 2u )
#define QD_HXC_HEADER_SIZE        40u
#define QD_DESCRIPTOR_OFFSET_FF   0x200u
#define QD_DESCRIPTOR_SIZE        16u
#define QD_MIN_TRACK_OFFSET       0x400u
#define QD_HEADER_DATA_SIZE       64u
#define QD_PHYSICAL_HEADER_SIZE   70u

static const uint8_t s_logical_start[4] = { 0x00, 0x16, 0x16, 0xa5 };
static const uint8_t s_logical_crc[3] = { 'C', 'R', 'C' };
static const uint8_t s_hxc_signature[8] = { 'H', 'X', 'C', 'Q', 'D', 'D', 'R', 'V' };

typedef struct qd_frame {
    size_t position;
    uint8_t type;
    size_t size;
    uint8_t *bytes;
} qd_frame_t;

typedef struct qd_frame_list {
    qd_frame_t *items;
    size_t count;
    size_t capacity;
} qd_frame_list_t;

static uint16_t read_le16 ( const uint8_t *p ) {
    return (uint16_t) ( (uint16_t) p[0] | ( (uint16_t) p[1] << 8 ) );
}

static uint32_t read_le32 ( const uint8_t *p ) {
    return (uint32_t) p[0]
         | ( (uint32_t) p[1] << 8 )
         | ( (uint32_t) p[2] << 16 )
         | ( (uint32_t) p[3] << 24 );
}

static int has_logical_start ( const uint8_t *p ) {
    return memcmp ( p, s_logical_start, sizeof ( s_logical_start ) ) == 0;
}

mz_qd_image_format_t mz_qd_image_detect ( const uint8_t *image, size_t image_size ) {
    if ( image == NULL ) return MZ_QD_IMAGE_FORMAT_UNKNOWN;

    if ( image_size >= sizeof ( s_hxc_signature )
      && memcmp ( image, s_hxc_signature, sizeof ( s_hxc_signature ) ) == 0 ) {
        return MZ_QD_IMAGE_FORMAT_HXC;
    }

    if ( image_size >= 5 && image[3] == 'Q' && image[4] == 'D' ) {
        return MZ_QD_IMAGE_FORMAT_FLASHFLOPPY;
    }

    if ( image_size >= 8
      && has_logical_start ( image )
      && memcmp ( image + 5, s_logical_crc, sizeof ( s_logical_crc ) ) == 0 ) {
        return MZ_QD_IMAGE_FORMAT_SHARP_LOGICAL;
    }

    return MZ_QD_IMAGE_FORMAT_UNKNOWN;
}

static mz_qd_image_error_t validate_logical ( const uint8_t *image, size_t image_size ) {
    size_t position = 8;
    unsigned block_count;
    unsigned file;

    if ( image_size < 8 ) return MZ_QD_IMAGE_ERROR_TRUNCATED;
    block_count = image[4];
    if ( ( block_count & 1u ) != 0 || block_count > QD_MAX_BLOCKS ) {
        return MZ_QD_IMAGE_ERROR_SEQUENCE;
    }

    for ( file = 0; file < block_count / 2u; ++file ) {
        uint16_t header_body_size;
        uint16_t body_size;

        if ( position > image_size || image_size - position < 74u ) {
            return MZ_QD_IMAGE_ERROR_TRUNCATED;
        }
        if ( !has_logical_start ( image + position )
          || image[position + 4] != 0x00
          || read_le16 ( image + position + 5 ) != QD_HEADER_DATA_SIZE
          || memcmp ( image + position + 71, s_logical_crc, 3 ) != 0 ) {
            return MZ_QD_IMAGE_ERROR_CORRUPT;
        }
        header_body_size = read_le16 ( image + position + 27 );
        position += 74u;

        if ( position > image_size || image_size - position < 10u ) {
            return MZ_QD_IMAGE_ERROR_TRUNCATED;
        }
        if ( !has_logical_start ( image + position ) || image[position + 4] != 0x05 ) {
            return MZ_QD_IMAGE_ERROR_CORRUPT;
        }
        body_size = read_le16 ( image + position + 5 );
        if ( body_size != header_body_size ) return MZ_QD_IMAGE_ERROR_SEQUENCE;
        if ( image_size - position < (size_t) body_size + 10u ) {
            return MZ_QD_IMAGE_ERROR_TRUNCATED;
        }
        if ( memcmp ( image + position + 7u + body_size, s_logical_crc, 3 ) != 0 ) {
            return MZ_QD_IMAGE_ERROR_CORRUPT;
        }
        position += (size_t) body_size + 10u;
    }

    return MZ_QD_IMAGE_OK;
}

static uint16_t qd_crc_update ( uint16_t crc, uint8_t data ) {
    unsigned bit;
    for ( bit = 0; bit < 8; ++bit ) {
        unsigned x = data & 1u;
        data >>= 1;
        if ( crc & 0x8000u ) x ^= 1u;
        crc = (uint16_t) ( crc << 1 );
        if ( x ) crc ^= 0x8005u;
    }
    return crc;
}

static int has_valid_crc ( const uint8_t *frame, size_t size ) {
    uint16_t crc = 0;
    size_t i;
    for ( i = 0; i < size; ++i ) crc = qd_crc_update ( crc, frame[i] );
    return crc == 0;
}

static int get_lsb_first_bit ( const uint8_t *bytes, size_t position ) {
    return ( bytes[position >> 3] & ( 1u << ( position & 7u ) ) ) != 0;
}

static uint8_t *decode_mfm_phase ( const uint8_t *track,
                                   size_t track_size,
                                   unsigned phase,
                                   size_t *decoded_size ) {
    size_t bit_count;
    size_t byte_count;
    size_t index;
    uint8_t *decoded;

    *decoded_size = 0;
    if ( track_size > ( (size_t) -1 ) / 8u ) return NULL;
    bit_count = track_size * 8u;
    if ( bit_count <= (size_t) phase + 14u ) return NULL;
    byte_count = ( bit_count - phase + 1u ) / 16u;
    decoded = (uint8_t*) malloc ( byte_count );
    if ( decoded == NULL ) return NULL;

    for ( index = 0; index < byte_count; ++index ) {
        size_t first_cell = (size_t) phase + index * 16u;
        uint8_t value = 0;
        unsigned bit;
        for ( bit = 0; bit < 8; ++bit ) {
            if ( get_lsb_first_bit ( track, first_cell + bit * 2u ) ) {
                value |= (uint8_t) ( 1u << bit );
            }
        }
        decoded[index] = value;
    }

    *decoded_size = byte_count;
    return decoded;
}

static int has_sharp_sync ( const uint8_t *decoded, size_t frame_index ) {
    size_t index = frame_index;
    size_t sync_count = 0;
    size_t lower_bound;

    while ( index > 0 && decoded[index - 1] == 0x16 ) {
        --index;
        ++sync_count;
    }
    if ( sync_count < 2 ) return 0;

    lower_bound = index > 17u ? index - 17u : 0u;
    while ( index > lower_bound ) {
        --index;
        if ( decoded[index] == 0x00 ) return 1;
    }
    return 0;
}

static void frame_list_free ( qd_frame_list_t *list ) {
    size_t i;
    for ( i = 0; i < list->count; ++i ) free ( list->items[i].bytes );
    free ( list->items );
    memset ( list, 0, sizeof ( *list ) );
}

static mz_qd_image_error_t frame_list_add ( qd_frame_list_t *list,
                                            size_t position,
                                            uint8_t type,
                                            const uint8_t *bytes,
                                            size_t size ) {
    size_t i;
    uint8_t *copy;

    for ( i = 0; i < list->count; ++i ) {
        const qd_frame_t *existing = &list->items[i];
        size_t distance = existing->position > position
                        ? existing->position - position : position - existing->position;
        if ( distance <= 15u && existing->size == size
          && memcmp ( existing->bytes, bytes, size ) == 0 ) {
            return MZ_QD_IMAGE_OK;
        }
    }

    if ( list->count == list->capacity ) {
        size_t new_capacity = list->capacity == 0 ? 8u : list->capacity * 2u;
        qd_frame_t *items;
        if ( new_capacity < list->capacity
          || new_capacity > ( (size_t) -1 ) / sizeof ( *items ) ) {
            return MZ_QD_IMAGE_ERROR_MEMORY;
        }
        items = (qd_frame_t*) realloc ( list->items, new_capacity * sizeof ( *items ) );
        if ( items == NULL ) return MZ_QD_IMAGE_ERROR_MEMORY;
        list->items = items;
        list->capacity = new_capacity;
    }

    copy = (uint8_t*) malloc ( size );
    if ( copy == NULL ) return MZ_QD_IMAGE_ERROR_MEMORY;
    memcpy ( copy, bytes, size );
    list->items[list->count].position = position;
    list->items[list->count].type = type;
    list->items[list->count].size = size;
    list->items[list->count].bytes = copy;
    ++list->count;
    return MZ_QD_IMAGE_OK;
}

static int compare_frames ( const void *left, const void *right ) {
    const qd_frame_t *a = (const qd_frame_t*) left;
    const qd_frame_t *b = (const qd_frame_t*) right;
    if ( a->position < b->position ) return -1;
    if ( a->position > b->position ) return 1;
    return 0;
}

static mz_qd_image_error_t find_physical_frames ( const uint8_t *track,
                                                  size_t track_size,
                                                  qd_frame_list_t *frames ) {
    unsigned phase;
    for ( phase = 0; phase < 16; ++phase ) {
        size_t decoded_size;
        uint8_t *decoded = decode_mfm_phase ( track, track_size, phase, &decoded_size );
        size_t index;
        if ( decoded == NULL ) {
            if ( decoded_size == 0
              && track_size <= ( (size_t) phase + 14u ) / 8u ) continue;
            return MZ_QD_IMAGE_ERROR_MEMORY;
        }

        for ( index = 0; index + 4u <= decoded_size; ++index ) {
            uint8_t marker;
            size_t frame_size;
            mz_qd_image_error_t error;

            if ( decoded[index] != 0xa5 || !has_sharp_sync ( decoded, index ) ) continue;
            marker = decoded[index + 1];

            if ( ( marker & 1u ) == 0
              && has_valid_crc ( decoded + index, 4u ) ) {
                error = frame_list_add ( frames, (size_t) phase + index * 16u,
                                         0x02, decoded + index, 4u );
                if ( error != MZ_QD_IMAGE_OK ) {
                    free ( decoded );
                    return error;
                }
                continue;
            }

            if ( marker != 0x00 && marker != 0x05 ) continue;
            {
                uint16_t data_size = read_le16 ( decoded + index + 2 );
                if ( marker == 0x00 && data_size != QD_HEADER_DATA_SIZE ) continue;
                frame_size = (size_t) data_size + 6u;
            }
            if ( frame_size > decoded_size - index ) continue;
            if ( !has_valid_crc ( decoded + index, frame_size ) ) continue;

            error = frame_list_add ( frames, (size_t) phase + index * 16u,
                                     marker, decoded + index, frame_size );
            if ( error != MZ_QD_IMAGE_OK ) {
                free ( decoded );
                return error;
            }
        }
        free ( decoded );
    }

    if ( frames->count > 1 ) {
        qsort ( frames->items, frames->count, sizeof ( frames->items[0] ), compare_frames );
    }
    return MZ_QD_IMAGE_OK;
}

static int is_blank_track ( const uint8_t *track, size_t track_size ) {
    size_t histogram[256] = { 0 };
    size_t dominant = 0;
    size_t required;
    size_t i;
    if ( track_size == 0 ) return 0;
    for ( i = 0; i < track_size; ++i ) ++histogram[track[i]];
    for ( i = 0; i < 256; ++i ) {
        if ( histogram[i] > dominant ) dominant = histogram[i];
    }
    required = ( track_size / 100u ) * 98u
             + ( ( track_size % 100u ) * 98u ) / 100u;
    return dominant >= required;
}

static mz_qd_image_error_t make_empty_logical ( uint8_t **output, size_t *output_size ) {
    uint8_t *logical = (uint8_t*) malloc ( 8u );
    if ( logical == NULL ) return MZ_QD_IMAGE_ERROR_MEMORY;
    memcpy ( logical, s_logical_start, 4u );
    logical[4] = 0;
    memcpy ( logical + 5, s_logical_crc, 3u );
    *output = logical;
    *output_size = 8u;
    return MZ_QD_IMAGE_OK;
}

static mz_qd_image_error_t build_logical_from_frames ( const qd_frame_list_t *frames,
                                                       uint8_t **output,
                                                       size_t *output_size ) {
    size_t count_index;
    mz_qd_image_error_t last_error = MZ_QD_IMAGE_ERROR_SEQUENCE;

    for ( count_index = 0; count_index < frames->count; ++count_index ) {
        const qd_frame_t *count_frame = &frames->items[count_index];
        const qd_frame_t *blocks[QD_MAX_BLOCKS];
        unsigned block_count;
        unsigned found = 0;
        size_t i;
        size_t logical_size = 8u;
        uint8_t *logical;
        size_t position;

        if ( count_frame->type != 0x02 ) continue;
        block_count = count_frame->bytes[1];
        if ( ( block_count & 1u ) != 0 || block_count > QD_MAX_BLOCKS ) {
            last_error = MZ_QD_IMAGE_ERROR_SEQUENCE;
            continue;
        }
        if ( block_count == 0 ) return make_empty_logical ( output, output_size );

        for ( i = count_index + 1; i < frames->count && found < block_count; ++i ) {
            if ( frames->items[i].position <= count_frame->position ) continue;
            if ( frames->items[i].type == 0x00 || frames->items[i].type == 0x05 ) {
                blocks[found++] = &frames->items[i];
            }
        }
        if ( found < block_count ) {
            last_error = MZ_QD_IMAGE_ERROR_SEQUENCE;
            continue;
        }

        for ( i = 0; i < block_count; i += 2u ) {
            uint16_t header_size;
            uint16_t body_size;
            if ( blocks[i]->type != 0x00 || blocks[i + 1u]->type != 0x05
              || blocks[i]->size != QD_PHYSICAL_HEADER_SIZE
              || blocks[i + 1u]->size < 6u ) {
                last_error = MZ_QD_IMAGE_ERROR_SEQUENCE;
                break;
            }
            header_size = read_le16 ( blocks[i]->bytes + 24u );
            body_size = read_le16 ( blocks[i + 1u]->bytes + 2u );
            if ( header_size != body_size
              || blocks[i + 1u]->size != (size_t) body_size + 6u ) {
                last_error = MZ_QD_IMAGE_ERROR_SEQUENCE;
                break;
            }
            if ( logical_size > ( (size_t) -1 ) - (size_t) body_size - 84u ) {
                return MZ_QD_IMAGE_ERROR_MEMORY;
            }
            logical_size += (size_t) body_size + 84u;
        }
        if ( i != block_count ) continue;

        logical = (uint8_t*) malloc ( logical_size );
        if ( logical == NULL ) return MZ_QD_IMAGE_ERROR_MEMORY;
        memcpy ( logical, s_logical_start, 4u );
        logical[4] = (uint8_t) block_count;
        memcpy ( logical + 5u, s_logical_crc, 3u );
        position = 8u;

        for ( i = 0; i < block_count; ++i ) {
            const qd_frame_t *frame = blocks[i];
            logical[position++] = 0x00;
            logical[position++] = 0x16;
            logical[position++] = 0x16;
            memcpy ( logical + position, frame->bytes, frame->size - 2u );
            position += frame->size - 2u;
            memcpy ( logical + position, s_logical_crc, 3u );
            position += 3u;
        }

        *output = logical;
        *output_size = logical_size;
        return MZ_QD_IMAGE_OK;
    }

    return last_error;
}

static mz_qd_image_error_t decode_physical ( const uint8_t *image,
                                             size_t image_size,
                                             mz_qd_image_format_t format,
                                             uint8_t **output,
                                             size_t *output_size ) {
    uint32_t descriptor_offset;
    uint32_t track_offset;
    uint32_t track_length;
    uint32_t window_start;
    uint32_t window_end;
    const uint8_t *descriptor;
    const uint8_t *track;
    qd_frame_list_t frames = { 0 };
    mz_qd_image_error_t error;

    if ( format == MZ_QD_IMAGE_FORMAT_HXC ) {
        uint32_t tracks;
        uint32_t sides;
        uint32_t encoding;
        if ( image_size < QD_HXC_HEADER_SIZE ) return MZ_QD_IMAGE_ERROR_TRUNCATED;
        tracks = read_le32 ( image + 12u );
        sides = read_le32 ( image + 16u );
        encoding = read_le32 ( image + 20u );
        if ( tracks != 1u || sides != 1u || encoding != 0u ) {
            return MZ_QD_IMAGE_ERROR_UNSUPPORTED;
        }
        descriptor_offset = read_le32 ( image + 36u );
    } else {
        descriptor_offset = QD_DESCRIPTOR_OFFSET_FF;
    }

    if ( descriptor_offset > image_size
      || image_size - descriptor_offset < QD_DESCRIPTOR_SIZE ) {
        return MZ_QD_IMAGE_ERROR_TRUNCATED;
    }
    descriptor = image + descriptor_offset;
    track_offset = read_le32 ( descriptor );
    track_length = read_le32 ( descriptor + 4u );
    window_start = read_le32 ( descriptor + 8u );
    window_end = read_le32 ( descriptor + 12u );

    if ( track_offset < QD_MIN_TRACK_OFFSET || track_length == 0u
      || track_offset > image_size || track_length > image_size - track_offset
      || window_start > window_end || window_end > track_length ) {
        return MZ_QD_IMAGE_ERROR_CORRUPT;
    }
    track = image + track_offset;

    error = find_physical_frames ( track, track_length, &frames );
    if ( error != MZ_QD_IMAGE_OK ) {
        frame_list_free ( &frames );
        return error;
    }
    if ( frames.count == 0 ) {
        error = is_blank_track ( track, track_length )
              ? make_empty_logical ( output, output_size )
              : MZ_QD_IMAGE_ERROR_CORRUPT;
    } else {
        error = build_logical_from_frames ( &frames, output, output_size );
    }
    frame_list_free ( &frames );
    return error;
}

mz_qd_image_error_t mz_qd_image_decode ( const uint8_t *image,
                                         size_t image_size,
                                         uint8_t **logical_image,
                                         size_t *logical_size,
                                         mz_qd_image_format_t *format ) {
    mz_qd_image_format_t detected;
    mz_qd_image_error_t error;

    if ( logical_image == NULL || logical_size == NULL ) return MZ_QD_IMAGE_ERROR_ARGUMENT;
    *logical_image = NULL;
    *logical_size = 0;
    if ( format != NULL ) *format = MZ_QD_IMAGE_FORMAT_UNKNOWN;
    if ( image == NULL || image_size == 0 ) return MZ_QD_IMAGE_ERROR_ARGUMENT;

    detected = mz_qd_image_detect ( image, image_size );
    if ( format != NULL ) *format = detected;
    if ( detected == MZ_QD_IMAGE_FORMAT_UNKNOWN ) return MZ_QD_IMAGE_ERROR_FORMAT;

    if ( detected == MZ_QD_IMAGE_FORMAT_SHARP_LOGICAL ) {
        uint8_t *copy;
        error = validate_logical ( image, image_size );
        if ( error != MZ_QD_IMAGE_OK ) return error;
        copy = (uint8_t*) malloc ( image_size );
        if ( copy == NULL ) return MZ_QD_IMAGE_ERROR_MEMORY;
        memcpy ( copy, image, image_size );
        *logical_image = copy;
        *logical_size = image_size;
        return MZ_QD_IMAGE_OK;
    }

    return decode_physical ( image, image_size, detected, logical_image, logical_size );
}

const char *mz_qd_image_error_string ( mz_qd_image_error_t error ) {
    switch ( error ) {
        case MZ_QD_IMAGE_OK:                return "no error";
        case MZ_QD_IMAGE_ERROR_ARGUMENT:    return "invalid decoder argument";
        case MZ_QD_IMAGE_ERROR_FORMAT:      return "unknown .qd format";
        case MZ_QD_IMAGE_ERROR_TRUNCATED:   return "truncated .qd image";
        case MZ_QD_IMAGE_ERROR_UNSUPPORTED: return "unsupported Quick Disk geometry or encoding";
        case MZ_QD_IMAGE_ERROR_CORRUPT:     return "corrupt Quick Disk container, frame, or CRC";
        case MZ_QD_IMAGE_ERROR_SEQUENCE:    return "inconsistent Quick Disk block sequence";
        case MZ_QD_IMAGE_ERROR_MEMORY:      return "not enough memory to decode .qd image";
        default:                            return "unknown .qd decoder error";
    }
}
