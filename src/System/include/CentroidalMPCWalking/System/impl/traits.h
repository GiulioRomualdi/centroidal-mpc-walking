/**
 * @file traits.h
 * @authors Giulio Romualdi
 * @copyright This software may be modified and distributed under the terms of the GNU Lesser
 * General Public License v2.1 or any later version.
 */

#ifndef CENTROIDAL_MPC_WALKING_SYSTEM_IMPL_TRAITS_H
#define CENTROIDAL_MPC_WALKING_SYSTEM_IMPL_TRAITS_H

/**
 * The user must call this macro before defining a custom System::Block
 */
#define CMW_DEFINE_SYSTEM_BLOCK_INTERAL_STRUCTURE(BlockType, InputType, OutputTime) \
    namespace CentroidalMPCWalking                                                  \
    {                                                                               \
    namespace System                                                                \
    {                                                                               \
    namespace internal                                                              \
    {                                                                               \
    template <> struct traits<BlockType>                                            \
    {                                                                               \
        using Input = InputType;                                                    \
        using Output = OutputTime;                                                  \
    };                                                                              \
    }                                                                               \
    }                                                                               \
    }

namespace CentroidalMPCWalking
{
namespace System
{
namespace internal
{

template <typename T> struct traits;

/// @note the following is from the Eigen library
/// here we say once and for all that traits<const T> == traits<T>
///
/// When constness must affect traits, it has to be constness on
/// template parameters on which T itself depends.
/// For example, traits<Map<const T> > != traits<Map<T> >, but
///              traits<const Map<T> > == traits<Map<T> >
template <typename T> struct traits<const T> : traits<T>
{
};

} // namespace internal
} // namespace System
} // namespace CentroildaMPCWalking

#endif // CENTROIDAL_MPC_WALKING_SYSTEM_IMPL_TRAITS_H
