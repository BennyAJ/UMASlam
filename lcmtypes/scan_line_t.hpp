/** THIS IS AN AUTOMATICALLY GENERATED FILE.  DO NOT MODIFY
 * BY HAND!!
 *
 * Generated by lcm-gen
 **/

#include <lcm/lcm_coretypes.h>

#ifndef __scan_line_t_hpp__
#define __scan_line_t_hpp__

#include <vector>
#include "point3D_t.hpp"


class scan_line_t
{
    public:
        int64_t    utime;

        int32_t    scan_size;

        std::vector< point3D_t > scan_line;

        std::vector< int8_t > hit;

    public:
        /**
         * Encode a message into binary form.
         *
         * @param buf The output buffer.
         * @param offset Encoding starts at thie byte offset into @p buf.
         * @param maxlen Maximum number of bytes to write.  This should generally be
         *  equal to getEncodedSize().
         * @return The number of bytes encoded, or <0 on error.
         */
        inline int encode(void *buf, int offset, int maxlen) const;

        /**
         * Check how many bytes are required to encode this message.
         */
        inline int getEncodedSize() const;

        /**
         * Decode a message from binary form into this instance.
         *
         * @param buf The buffer containing the encoded message.
         * @param offset The byte offset into @p buf where the encoded message starts.
         * @param maxlen The maximum number of bytes to reqad while decoding.
         * @return The number of bytes decoded, or <0 if an error occured.
         */
        inline int decode(const void *buf, int offset, int maxlen);

        /**
         * Retrieve the 64-bit fingerprint identifying the structure of the message.
         * Note that the fingerprint is the same for all instances of the same
         * message type, and is a fingerprint on the message type definition, not on
         * the message contents.
         */
        inline static int64_t getHash();

        /**
         * Returns "scan_line_t"
         */
        inline static const char* getTypeName();

        // LCM support functions. Users should not call these
        inline int _encodeNoHash(void *buf, int offset, int maxlen) const;
        inline int _getEncodedSizeNoHash() const;
        inline int _decodeNoHash(const void *buf, int offset, int maxlen);
        inline static int64_t _computeHash(const __lcm_hash_ptr *p);
};

int scan_line_t::encode(void *buf, int offset, int maxlen) const
{
    int pos = 0, tlen;
    int64_t hash = getHash();

    tlen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &hash, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = this->_encodeNoHash(buf, offset + pos, maxlen - pos);
    if (tlen < 0) return tlen; else pos += tlen;

    return pos;
}

int scan_line_t::decode(const void *buf, int offset, int maxlen)
{
    int pos = 0, thislen;

    int64_t msg_hash;
    thislen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &msg_hash, 1);
    if (thislen < 0) return thislen; else pos += thislen;
    if (msg_hash != getHash()) return -1;

    thislen = this->_decodeNoHash(buf, offset + pos, maxlen - pos);
    if (thislen < 0) return thislen; else pos += thislen;

    return pos;
}

int scan_line_t::getEncodedSize() const
{
    return 8 + _getEncodedSizeNoHash();
}

int64_t scan_line_t::getHash()
{
    static int64_t hash = _computeHash(NULL);
    return hash;
}

const char* scan_line_t::getTypeName()
{
    return "scan_line_t";
}

int scan_line_t::_encodeNoHash(void *buf, int offset, int maxlen) const
{
    int pos = 0, tlen;

    tlen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &this->utime, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int32_t_encode_array(buf, offset + pos, maxlen - pos, &this->scan_size, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    for (int a0 = 0; a0 < this->scan_size; a0++) {
        tlen = this->scan_line[a0]._encodeNoHash(buf, offset + pos, maxlen - pos);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    if(this->scan_size > 0) {
        tlen = __int8_t_encode_array(buf, offset + pos, maxlen - pos, &this->hit[0], this->scan_size);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    return pos;
}

int scan_line_t::_decodeNoHash(const void *buf, int offset, int maxlen)
{
    int pos = 0, tlen;

    tlen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &this->utime, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, &this->scan_size, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    this->scan_line.resize(this->scan_size);
    for (int a0 = 0; a0 < this->scan_size; a0++) {
        tlen = this->scan_line[a0]._decodeNoHash(buf, offset + pos, maxlen - pos);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    if(this->scan_size) {
        this->hit.resize(this->scan_size);
        tlen = __int8_t_decode_array(buf, offset + pos, maxlen - pos, &this->hit[0], this->scan_size);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    return pos;
}

int scan_line_t::_getEncodedSizeNoHash() const
{
    int enc_size = 0;
    enc_size += __int64_t_encoded_array_size(NULL, 1);
    enc_size += __int32_t_encoded_array_size(NULL, 1);
    for (int a0 = 0; a0 < this->scan_size; a0++) {
        enc_size += this->scan_line[a0]._getEncodedSizeNoHash();
    }
    enc_size += __int8_t_encoded_array_size(NULL, this->scan_size);
    return enc_size;
}

int64_t scan_line_t::_computeHash(const __lcm_hash_ptr *p)
{
    const __lcm_hash_ptr *fp;
    for(fp = p; fp != NULL; fp = fp->parent)
        if(fp->v == scan_line_t::getHash)
            return 0;
    const __lcm_hash_ptr cp = { p, (void*)scan_line_t::getHash };

    int64_t hash = 0xc558f9306aa44b9dLL +
         point3D_t::_computeHash(&cp);

    return (hash<<1) + ((hash>>63)&1);
}

#endif
