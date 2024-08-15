#include "hand_coding.h"
#include "hand_data/hand_data.h"

/**
 * @brief For Nanopb, the parameter is wrapped in an additional `const` pointer
 * when passed into the function. Therefore, we need to first dereference `*arg`
 * to obtain a `void*`, and then cast it to the specified type.
 */

/* public API */

bool hand_encode_float_array(pb_ostream_t *stream, const pb_field_t *field,
                             void *const *arg)
{
  hand_float_arr_arg_t *data_arg = (hand_float_arr_arg_t *)(*arg);
  return pb_encode_tag_for_field(stream, field) &&
         pb_encode_string(stream, (uint8_t *)data_arg->fp,
                          data_arg->count * sizeof(float));
}

/* TODO: decode_float_array */

bool hand_encode_timestamps_array(pb_ostream_t *stream, const pb_field_t *field,
                                  void *const *arg)
{
  hand_timestamps_arr_arg_t *t_arg = (hand_timestamps_arr_arg_t *)(*arg);
  for (int i = 0; i < t_arg->count; i++)
  {
    if (!pb_encode_tag_for_field(stream, field))
    {
      return false;
    }
    if (!pb_encode_varint(stream, (t_arg->ts_p)[i]))
    {
      return false;
    }
  }

  return true;
}

/* TODO: decode_timestamps_array */

bool hand_encode_data_msg_pointers_array(pb_ostream_t *stream,
                                         const pb_field_t *field,
                                         void *const *arg)
{
  hand_data_msgs_arr_arg_t *msg_arg = (hand_data_msgs_arr_arg_t *)(*arg);
  /* Important:
    HandDataMsg *msg_p = *(msg_arg->msgs_pp);
    &msg_p[i] != *(msg_arg->msgs_pp + i) when i != 0

    i   &msg_p[i]      *(msg_arg->msgs_pp + i)
    0: 0x3fcbded0   ==      0x3fcbded0
    1: 0x3fcbdef8   ==      0x3fcbdea8

    use `*(msg_arg->msgs_pp + i)` or` msg_arg->msgs_pp[i]` to express address
    instead

    Note: Due to stack allocation order, data_msg[i+1] may not be located 
    exactly at data_msg[i] + offset.

    Original: HandDataMsg *msg_p = *(msg_arg->msgs_pp); is wrong!!!
   */
  for (int i = 0; i < msg_arg->count; i++)
  {
    if (!pb_encode_tag_for_field(stream, field)) return false;
    if (!pb_encode_submessage(stream, HandDataMsg_fields, msg_arg->msgs_pp[i]))
      return false;
  }
  return true;
}

bool hand_overwrite_buf_bytes_count(uint8_t *buf, uint32_t bytes_count)
{
  /* make the 1-4th bytes becomes new ostream to overwrite buffer bytes count
   * content */
  pb_ostream_t stream =
      pb_ostream_from_buffer(buf + HAND_MSG_BYTES_COUNT_VALUE_OFFSET,
                             HAND_MSG_BYTES_COUNT_VALUE_LENGTH);

  return pb_encode_fixed32(&stream, &bytes_count);
}