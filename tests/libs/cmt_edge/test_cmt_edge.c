#include "mztest.h"

#include <stdio.h>
#include <stdlib.h>

#include "emulator/hw-generic/cmt/cmt_edge.h"
#include "emulator/hw-generic/cmt/cmtext.h"
#include "hw-generic/gdg/gdgclk.h"

static const char *g_temp_path = "test_cmt_edge_roundtrip.lep";

void setUp(void)
{
    remove(g_temp_path);
}

void tearDown(void)
{
    remove(g_temp_path);
}

static void assert_pulse(st_CMT_STREAM *stream, uint64_t expected_units, int expected_value)
{
    uint64_t units = 0;
    int value = -1;
    TEST_ASSERT_EQUAL_INT(EXIT_SUCCESS,
                          cmt_vstream_read_pulse(stream->str.vstream, &units, &value));
    TEST_ASSERT_EQUAL_UINT64(expected_units, units);
    TEST_ASSERT_EQUAL_INT(expected_value, value);
}

static void test_decode_lep_runs_and_continuation(void)
{
    const uint8_t bytes[] = { 5, (uint8_t) (int8_t) -3, 0, 2 };
    st_CMT_STREAM *stream = cmt_edge_stream_from_data(bytes, sizeof(bytes),
                                                      CMT_EDGE_LEP_RATE,
                                                      CMT_STREAM_POLARITY_NORMAL);
    TEST_ASSERT_NOT_NULL(stream);
    TEST_ASSERT_EQUAL_UINT32(CMT_EDGE_LEP_RATE, cmt_stream_get_rate(stream));
    TEST_ASSERT_EQUAL_UINT64(137, cmt_stream_get_count_scans(stream));

    cmt_vstream_read_reset(stream->str.vstream);
    /* LEP/L16 signs are physical connector levels.  Sharp's interface
     * inverts them before 8255 PC5, represented here by the stream value. */
    assert_pulse(stream, 5, 0);
    assert_pulse(stream, 130, 1);
    assert_pulse(stream, 2, 0);
    uint64_t units;
    int value;
    TEST_ASSERT_EQUAL_INT(EXIT_FAILURE,
                          cmt_vstream_read_pulse(stream->str.vstream, &units, &value));
    cmt_stream_destroy(stream);
}

static void test_decode_rejects_empty_and_initial_zero(void)
{
    const uint8_t zero[] = { 0 };
    TEST_ASSERT_NULL(cmt_edge_stream_from_data(NULL, 0, CMT_EDGE_LEP_RATE,
                                               CMT_STREAM_POLARITY_NORMAL));
    TEST_ASSERT_NULL(cmt_edge_stream_from_data(zero, sizeof(zero), CMT_EDGE_LEP_RATE,
                                               CMT_STREAM_POLARITY_NORMAL));
}

static void test_decode_accepts_reference_int8_min(void)
{
    const uint8_t bytes[] = { 0x80 };
    st_CMT_STREAM *stream = cmt_edge_stream_from_data(bytes, sizeof(bytes),
                                                      CMT_EDGE_L16_RATE,
                                                      CMT_STREAM_POLARITY_NORMAL);
    TEST_ASSERT_NOT_NULL(stream);
    cmt_vstream_read_reset(stream->str.vstream);
    assert_pulse(stream, 128, 1);
    cmt_stream_destroy(stream);
}

static void test_decode_applies_inverted_polarity(void)
{
    const uint8_t bytes[] = { 4, (uint8_t) (int8_t) -2 };
    st_CMT_STREAM *stream = cmt_edge_stream_from_data(bytes, sizeof(bytes),
                                                      CMT_EDGE_LEP_RATE,
                                                      CMT_STREAM_POLARITY_INVERTED);
    TEST_ASSERT_NOT_NULL(stream);
    cmt_vstream_read_reset(stream->str.vstream);
    assert_pulse(stream, 4, 1);
    assert_pulse(stream, 2, 0);
    cmt_stream_destroy(stream);
}

static void assert_encoded_run(int value, uint64_t units,
                               const uint8_t *expected, size_t expected_size)
{
    uint8_t output[8] = { 0 };
    size_t written = 0;
    TEST_ASSERT_EQUAL_INT(EXIT_SUCCESS,
                          cmt_edge_encode_run(value, units, output,
                                              sizeof(output), &written));
    TEST_ASSERT_EQUAL_UINT(expected_size, written);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, output, expected_size);
    TEST_ASSERT_NOT_EQUAL(0, output[0]);
}

static void test_encode_canonical_long_runs(void)
{
    const uint8_t high_300[] = { 46, 0, 0 };
    const uint8_t low_300[] = {
        (uint8_t) (int8_t) -46, 0, 0
    };
    const uint8_t high_254[] = { 127, 0 };
    const uint8_t high_255[] = { 1, 0, 0 };
    assert_encoded_run(1, 300, high_300, sizeof(high_300));
    assert_encoded_run(0, 300, low_300, sizeof(low_300));
    assert_encoded_run(1, 254, high_254, sizeof(high_254));
    assert_encoded_run(1, 255, high_255, sizeof(high_255));
}

static void test_encode_rejects_invalid_arguments(void)
{
    uint8_t output[1];
    size_t written = 99;
    TEST_ASSERT_EQUAL_INT(EXIT_FAILURE,
                          cmt_edge_encode_run(1, 0, output, sizeof(output), &written));
    TEST_ASSERT_EQUAL_UINT(0, written);
    TEST_ASSERT_EQUAL_INT(EXIT_FAILURE,
                          cmt_edge_encode_run(1, 128, output, sizeof(output), &written));
}

static void test_file_roundtrip_preserves_pulses(void)
{
    const uint8_t canonical[] = { 46, 0, 0, (uint8_t) (int8_t) -7, 9 };
    st_CMT_STREAM *stream = cmt_stream_new(CMT_STREAM_TYPE_VSTREAM);
    TEST_ASSERT_NOT_NULL(stream);
    stream->str.vstream = cmt_vstream_new(CMT_EDGE_LEP_RATE,
                                          CMT_VSTREAM_BYTELENGTH8,
                                          1,
                                          CMT_STREAM_POLARITY_NORMAL);
    TEST_ASSERT_NOT_NULL(stream->str.vstream);
    TEST_ASSERT_EQUAL_INT(EXIT_SUCCESS,
                          cmt_vstream_add_value(stream->str.vstream, 1, 300));
    TEST_ASSERT_EQUAL_INT(EXIT_SUCCESS,
                          cmt_vstream_add_value(stream->str.vstream, 0, 7));
    TEST_ASSERT_EQUAL_INT(EXIT_SUCCESS,
                          cmt_vstream_add_value(stream->str.vstream, 1, 9));
    TEST_ASSERT_EQUAL_INT(EXIT_SUCCESS,
                          cmt_edge_stream_write_file(stream, g_temp_path));

    FILE *fh = fopen(g_temp_path, "rb");
    TEST_ASSERT_NOT_NULL(fh);
    uint8_t saved[16] = { 0 };
    size_t saved_size = fread(saved, 1, sizeof(saved), fh);
    fclose(fh);
    TEST_ASSERT_EQUAL_UINT(sizeof(canonical), saved_size);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(canonical, saved, sizeof(canonical));

    st_CMT_STREAM *loaded = cmt_edge_stream_from_data(saved, saved_size,
                                                      CMT_EDGE_LEP_RATE,
                                                      CMT_STREAM_POLARITY_NORMAL);
    TEST_ASSERT_NOT_NULL(loaded);
    cmt_vstream_read_reset(loaded->str.vstream);
    assert_pulse(loaded, 300, 0);
    assert_pulse(loaded, 7, 1);
    assert_pulse(loaded, 9, 0);
    cmt_stream_destroy(loaded);
    cmt_stream_destroy(stream);
}

static void test_tick_conversion_has_no_cumulative_drift(void)
{
    int ok = 0;
    uint64_t seconds = 60u * 60u * 24u;
    uint64_t ticks = (uint64_t) GDGCLK_BASE * seconds;
    TEST_ASSERT_EQUAL_UINT64(seconds * CMT_EDGE_LEP_RATE,
                             cmt_edge_ticks_to_units(ticks, CMT_EDGE_LEP_RATE, &ok));
    TEST_ASSERT_TRUE(ok);
    TEST_ASSERT_EQUAL_UINT64(seconds * CMT_EDGE_L16_RATE,
                             cmt_edge_ticks_to_units(ticks, CMT_EDGE_L16_RATE, &ok));
    TEST_ASSERT_TRUE(ok);
}

static void test_recorder_uses_native_rate_and_edge_intervals(void)
{
    TEST_ASSERT_EQUAL_INT(EXIT_SUCCESS,
                          g_cmt_edge_save_extension.cb_open((char *) g_temp_path));
    TEST_ASSERT_NOT_NULL(g_cmt_edge_save_extension.block);

    g_cmt_edge_save_extension.cb_write((uint64_t) GDGCLK_BASE, 1);
    g_cmt_edge_save_extension.cb_write((uint64_t) GDGCLK_BASE * 2u, 0);

    st_CMT_STREAM *stream = g_cmt_edge_save_extension.block->stream;
    TEST_ASSERT_NOT_NULL(stream);
    TEST_ASSERT_EQUAL_UINT32(CMT_EDGE_LEP_RATE, cmt_stream_get_rate(stream));
    cmt_vstream_read_reset(stream->str.vstream);
    assert_pulse(stream, CMT_EDGE_LEP_RATE, 0);
    assert_pulse(stream, CMT_EDGE_LEP_RATE, 1);

    g_cmt_edge_save_extension.cb_eject();
}

static void test_recorder_quantizes_each_halfwave_independently(void)
{
    TEST_ASSERT_EQUAL_INT(EXIT_SUCCESS,
                          g_cmt_edge_save_extension.cb_open((char *) g_temp_path));
    TEST_ASSERT_NOT_NULL(g_cmt_edge_save_extension.block);

    /*
     * 37/8 = 4.625 LEP units per half-wave (~231.25 us).  Each individual
     * interval rounds to 5 units.  The old absolute/cumulative quantizer
     * error-diffused this constant-width waveform as 5,4,5,5,4,... .
     *
     * For an edge-duration tape format that is undesirable: a Sharp loader
     * samples one half-wave at a time, so constant physical widths must remain
     * constant after quantization whenever they round to the same LEP slot.
     */
    uint64_t delta_ticks =
        ((uint64_t) GDGCLK_BASE * 37u) /
        ((uint64_t) CMT_EDGE_LEP_RATE * 8u);

    TEST_ASSERT_EQUAL_UINT64(
        5u, cmt_edge_ticks_to_units(delta_ticks, CMT_EDGE_LEP_RATE, NULL));

    for (uint64_t edge = 1; edge <= 8; ++edge) {
        g_cmt_edge_save_extension.cb_write(delta_ticks * edge, (int) (edge & 1u));
    }

    st_CMT_STREAM *stream = g_cmt_edge_save_extension.block->stream;
    TEST_ASSERT_NOT_NULL(stream);
    cmt_vstream_read_reset(stream->str.vstream);

    for (int pulse = 0; pulse < 8; ++pulse)
        assert_pulse(stream, 5, pulse & 1);

    g_cmt_edge_save_extension.cb_eject();
}

static void test_recording_extension_selection(void)
{
    st_CMTEXT *wav = cmtext_get_recording_extension_for_filename("capture.wav");
    st_CMTEXT *lep = cmtext_get_recording_extension_for_filename("capture.LEP");
    st_CMTEXT *l16 = cmtext_get_recording_extension_for_filename("capture.l16");
    TEST_ASSERT_NOT_NULL(wav);
    TEST_ASSERT_NOT_NULL(lep);
    TEST_ASSERT_NOT_NULL(l16);
    TEST_ASSERT_EQUAL_STRING("SAVE-WAV", cmtext_get_name(wav));
    TEST_ASSERT_EQUAL_STRING("SAVE-LEP/L16", cmtext_get_name(lep));
    TEST_ASSERT_EQUAL_PTR(lep, l16);
    TEST_ASSERT_EQUAL_PTR(wav, cmtext_get_recording_extension_for_filename("capture"));
    TEST_ASSERT_NULL(cmtext_get_recording_extension_for_filename("capture.unknown"));
}

int main(int argc, char *argv[])
{
    mztest_parse_args(argc, argv);
    mztest_init();

    UNITY_BEGIN();
    RUN_TEST(test_decode_lep_runs_and_continuation);
    RUN_TEST(test_decode_rejects_empty_and_initial_zero);
    RUN_TEST(test_decode_accepts_reference_int8_min);
    RUN_TEST(test_decode_applies_inverted_polarity);
    RUN_TEST(test_encode_canonical_long_runs);
    RUN_TEST(test_encode_rejects_invalid_arguments);
    RUN_TEST(test_file_roundtrip_preserves_pulses);
    RUN_TEST(test_tick_conversion_has_no_cumulative_drift);
    RUN_TEST(test_recorder_uses_native_rate_and_edge_intervals);
    RUN_TEST(test_recorder_quantizes_each_halfwave_independently);
    RUN_TEST(test_recording_extension_selection);
    int result = UNITY_END();

    mztest_teardown();
    return result;
}
