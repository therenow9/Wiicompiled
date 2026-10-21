#include "runtime_product.h"

namespace RuntimeProduct {

const Descriptor& Active() noexcept {
    static constexpr Descriptor descriptor{
        Kind::BaseGame,
        "Mario Kart Wii 8 Player Splitscreen",
    };
    return descriptor;
}

} // namespace RuntimeProduct
