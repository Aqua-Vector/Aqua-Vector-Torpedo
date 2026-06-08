#ifndef TORPEDO_PARSER_HPP_
#define TORPEDO_PARSER_HPP_

#include "GenericParser.hpp"
#include "ProtocolPolicies.hpp"
#include "GenericPacket.hpp"
#include "Payloads.hpp"

using TorpedoParserBase = GenericParser<TorpedoPolicy>;

class TorpedoParser : public TorpedoParserBase {
public:
    using TorpedoParserBase::TorpedoParserBase;
    using TorpedoParserBase::serialize;

    // GenericPacket<TorpedoUplinkPayload, uint16_t> (Sync 0xBB) 직렬화 지원
    size_t serialize(const GenericPacket<TorpedoUplinkPayload, uint16_t>& pkt, uint8_t* buf, size_t max_len) {
        if (max_len < sizeof(GenericPacket<TorpedoUplinkPayload, uint16_t>)) return 0;
        std::memcpy(buf, &pkt, sizeof(GenericPacket<TorpedoUplinkPayload, uint16_t>));
        return sizeof(GenericPacket<TorpedoUplinkPayload, uint16_t>);
    }
};

#endif /* TORPEDO_PARSER_HPP_ */
