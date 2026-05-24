#ifndef TORPEDO_PARSER_HPP_
#define TORPEDO_PARSER_HPP_

#include "GenericParser.hpp"
#include "ProtocolPolicies.hpp"
#include "UplinkPacket.hpp"

using TorpedoParserBase = GenericParser<TorpedoPolicy>;

class TorpedoParser : public TorpedoParserBase {
public:
    using TorpedoParserBase::TorpedoParserBase;
    using TorpedoParserBase::serialize;

    // UplinkPacket (Sync 0xBB) 직렬화 지원
    size_t serialize(const UplinkPacket& pkt, uint8_t* buf, size_t max_len) {
        if (max_len < sizeof(UplinkPacket)) return 0;
        std::memcpy(buf, &pkt, sizeof(UplinkPacket));
        return sizeof(UplinkPacket);
    }
};

#endif /* TORPEDO_PARSER_HPP_ */
