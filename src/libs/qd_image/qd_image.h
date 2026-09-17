/*
 * qd_image.h - read-only importer for Sharp Quick Disk .qd images
 *
 * Supports the legacy logical Sharp/MZ stream and the physical HxC and
 * FlashFloppy containers.  The decoder returns the logical byte stream used
 * internally by the MZ-1F11 emulation (the same framing as an .mzq image).
 */

#ifndef MZ_QD_IMAGE_H
#define MZ_QD_IMAGE_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum mz_qd_image_format {
    MZ_QD_IMAGE_FORMAT_UNKNOWN = 0,
    MZ_QD_IMAGE_FORMAT_SHARP_LOGICAL,
    MZ_QD_IMAGE_FORMAT_HXC,
    MZ_QD_IMAGE_FORMAT_FLASHFLOPPY
} mz_qd_image_format_t;

typedef enum mz_qd_image_error {
    MZ_QD_IMAGE_OK = 0,
    MZ_QD_IMAGE_ERROR_ARGUMENT,
    MZ_QD_IMAGE_ERROR_FORMAT,
    MZ_QD_IMAGE_ERROR_TRUNCATED,
    MZ_QD_IMAGE_ERROR_UNSUPPORTED,
    MZ_QD_IMAGE_ERROR_CORRUPT,
    MZ_QD_IMAGE_ERROR_SEQUENCE,
    MZ_QD_IMAGE_ERROR_MEMORY
} mz_qd_image_error_t;

/** Detect a supported .qd representation from its contents. */
mz_qd_image_format_t mz_qd_image_detect ( const uint8_t *image, size_t image_size );

/**
 * Validate and decode a .qd image into an allocated logical MZQ byte stream.
 *
 * The returned buffer is allocated with malloc() and must be released with
 * free().  Output arguments are cleared on failure.
 */
mz_qd_image_error_t mz_qd_image_decode ( const uint8_t *image,
                                         size_t image_size,
                                         uint8_t **logical_image,
                                         size_t *logical_size,
                                         mz_qd_image_format_t *format );

/** Stable English diagnostic for a decoder result. */
const char *mz_qd_image_error_string ( mz_qd_image_error_t error );

#ifdef __cplusplus
}
#endif

#endif /* MZ_QD_IMAGE_H */
