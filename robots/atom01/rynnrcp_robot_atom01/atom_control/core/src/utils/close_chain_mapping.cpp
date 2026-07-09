#include "utils/close_chain_mapping.hpp"
#include "utils/decouple_rpo.hpp"

std::shared_ptr<Decouple> Decouple::create(const std::string &type)
{
    if (type == "rpo")
    {
        return std::make_shared<DecoupleRPO>();
    }
    else
    {
        throw std::runtime_error("Unknown close_chain type: " + type +
                                 ". Supported types: rpo");
    }
}
