// Copyright (c) RVBUST, Inc - All rights reserved.
#pragma once

#include <RVS/CollisionChecker/Types.h>

namespace RVS
{
/// @addtogroup CollisionChecker
/// @{

enum StackDimension
{
    StackDimensionX = 0,
    StackDimensionY = 1,
    StackDimensionZ = 2

};

/**
 * @brief This is a template class that holds a series of voxels in a stack. It
 * is the core of distance field that uniformly manage and dispatch the
 * generation, update and destruction of field
 *
 * @tparam VType The type of a single voxel
 */
template <typename VType>
class VoxelStack
{
public:
    // default constructor
    VoxelStack();

    /**
     * @brief Construct a new voxel stack object
     *
     * @param stack_bottom_left  The coordinate of bottom left vertex of
     * (rectangular) cuboid
     * @param stack_top_right The coordinate of top right vertex of cuboid
     * @param resolution  The distance of adjacent two voxel (unit: meter)
     *
     */
    VoxelStack(const std::array<double, 3> &stack_bottom_left,
               const std::array<double, 3> &stack_top_right, double resolution);

    // default destructor
    virtual ~VoxelStack(){};

    /**
     * @brief Set the range of stack
     *
     * @param stack_bottom_left Similarly above
     * @param stack_top_right Similarly above
     *
     */
    void SetStackRange(const std::array<double, 3> &stack_bottom_left,
                       const std::array<double, 3> &stack_top_right);


    /**
     * @brief Get the size of stack (cuboid)
     *
     * @return const std::array<double, 3>& The three components of the return
     * array are corresponding to length, width and height of cuboid stack
     * respectively
     *
     */
    const std::array<double, 3> &GetStackSize() const;

    /**
     * @brief Get the reference point of the stack. Its definition is
     * left bottom vertex of cuboid stack
     *
     * @return const std::array<double, 3>& The x,y and z coordinate of
     * reference point of stack
     */
    std::array<double, 3> GetReferPoint() const;

    /**
     * @brief Set the resolution (also called sampling interval) of stack
     *
     * @param resolution Similarly above
     */
    void SetStackResolution(double resolution);

    /**
     * @brief Get the resolution of stack
     *
     * @return double The current resolution of stack
     *
     */
    double GetStackResolution() const;

    /**
     * @brief Reset all voxels in a voxel stack with the given voxel
     *
     * @param ref_voxel  The new value of all voxels in the stack
     *
     */
    void ResetVoxelStack(const VType &ref_voxel);

    /**
     * @brief To initialize the related quantities of stack. More often
     * than not, the function should be invoked before performing any task with
     * voxels in order to synchronize the latest parameters (range of stack or
     * resolution) into voxel stack
     *
     * @return true The result if it succeeds in initializing voxel stack
     * @return false  The result if it fails to do it
     *
     */
    bool InitializeVoxelStack();

    /**
     * @brief Get the voxel object
     *
     * @param voxel_idx The 3-dimension index of voxel in a stack
     * @return VType& The voxel in given 3d index
     *
     */

    VType &GetVoxel(const std::array<int, 3> &voxel_idx);

    /**
     * @brief Get the voxel object
     *
     * @param i_x The index along x-axis in a stack
     * @param i_y The index along y-axis in a stack
     * @param i_z The index along z-axis in a stack
     * @return VType&  The voxel in given 3d index
     *
     */
    VType &GetVoxel(int i_x, int i_y, int i_z);

    /**
     * @brief Get the number of voxels along specified reference direction (X,Y
     or Z)
     *
     * @param stack_dim Specify direction of stack. (The stack
     * direction consists of StackDimensionX, StackDimensionY and
     * StackDimensionZ)
     * @return int The number of voxels along given direction
     *
     */
    int GetNumOfVoxelsAtDim(StackDimension stack_dim) const;

    /**
     * @brief Set the new value of voxel in given 3d index
     *
     * @param i_x The index along x-axis direction
     * @param i_y The index along y-axis direction
     * @param i_z The index along z-axis direction
     * @param voxel The new voxel that will be assigned into given index
     * position
     * @return true The result if it succeeds to update the voxel in given index
     * @return false The result if it fails to do it
     */
    bool UpdateVoxel(int i_x, int i_y, int i_z, const VType &voxel);

    /**
     * @brief Set the new value of voxel in given 3d index
     *
     * @param voxel_idx Similarly above
     * @param voxel  Similarly above
     * @return true Similarly above
     * @return false Similarly above
     *
     */
    bool UpdateVoxel(const std::array<int, 3> &voxel_idx, const VType &voxel);

    /**
     * @brief To implement transformation from 3d index of voxel to its
     * world coordinate in a stack
     *
     * @param voxel_idx The input 3d index of voxel
     * @param coord The output 3d coordinate of voxel
     * @return true The result if the input 3d index lies in the voxel stack
     * @return false The result if invalid 3d index
     *
     */
    bool IndexToCoord(const std::array<int, 3> &voxel_idx,
                      std::array<double, 3> &coord) const;

    /**
     * @brief To implement transformation from 3d index of voxel to its
     * world coordinate in a stack
     * @note Please refer to above IndexToCoord(..) for the meaning of
     * parameters
     *
     */
    bool IndexToCoord(int i_x, int i_y, int i_z, double &coord_x,
                      double &coord_y, double &coord_z) const;

    /**
     * @brief To implement transformation from the world coordinate of the voxel
     * to its 3d index in a stack
     *
     * @param coord The input world coordinate of voxel
     * @param voxel_idx The output 3d index of voxel
     * @return true The result if there exists the intersection between input
     * world coordinate and the voxel stack
     * @return false The result if invalid input world coordinate of voxel
     *
     */
    bool CoordToIndex(const std::array<double, 3> &coord,
                      std::array<int, 3> &voxel_idx) const;

    /**
     * @brief To implement transformation from the world coordinate of the voxel
     * to its 3d index in a stack
     * @note Please refer to above CoordToIndex(...) for the meaning of
     * parameters
     *
     */
    bool CoordToIndex(double coord_x, double coord_y, double coord_z, int &i_x,
                      int &i_y, int &i_z) const;

    /**
     * @brief To check whether the given 3d index of voxel lies in voxel stack
     *
     * @param voxel_idx The querying 3d index of voxel
     * @return true  The result if the querying 3d index exactly populates the
     * voxel stack
     * @return false The result if invalid 3d index
     */
    bool IsValidVoxel(const std::array<int, 3> &voxel_idx) const;

    /**
     * @brief To check whether the given 3d index of voxel lies in voxel stack
     *
     * @note Please refer to above IsValidVoxel(...) for the meaning of
     * paramters
     */
    bool IsValidVoxel(int i_x, int i_y, int i_z) const;

    /**
     * @brief To determine if stack has initialized successfully
     *
     * @return true The result if the voxel stack has been initialized
     * @return false The result if the voxel stack has not been initialized
     *
     */
    bool HasInitializedStack() const;


private:
    // determine whether the current stack is valid
    bool _IsRangeOfStackValid();

    // determine whether current sampling interval of stack is valid
    bool _IsResolutionValid(double resolution) const;

    // map 3d index of voxel in voxel stack into 1d index of array that actually
    // stores all voxels
    int _MappingIndexFrom3DTo1D(const int &i_x, const int &i_y,
                                const int &i_z) const;

    // Obtain the 3d index of voxel according to given world coordinate of voxel
    std::array<int, 3>
    _GetVoxelIdxFromCoord(const std::array<double, 3> &coord) const;

    // Obtain the world coordinate of voxel according to given 3d index of voxel
    std::array<double, 3>
    _GetVoxelCoordFromIdx(const std::array<int, 3> &voxel_idx) const;

    std::vector<std::shared_ptr<VType>>
        m_voxels; ///< the set of all voxels in voxel stack
    std::array<double, 3> m_stack_size; ///< the size of cuboid stack
    double m_resolution; ///< the resolution of stack
    static const double m_resolution_lower_bound;
    static const double m_resolution_upper_bound;
    std::array<double, 3> m_bottom_left_vertex; ///< the left bottom vertex of
                                                ///< cuboid stack
    std::array<double, 3>
        m_top_right_vertex; ///< the right top vertex of cuboid stack
    std::array<int, 3>
        m_num_of_voxels; ///< the number of voxels along x,y and z-axis
                         ///< direction of the voxel stack respectively
    size_t m_total_voxels; ///< the total amounts of voxels in voxel stack
    int m_stride_x; ///< The number of voxels of each level of the voxel stack
                    ///< along x-axis direction;
    int m_stride_y; ///< The number of voxels along y-axis direction of each
                    ///< level of the voxel stack along x-axis direction
    static VType m_uninitialized_voxel;
};

template <typename VType>
VType VoxelStack<VType>::m_uninitialized_voxel = VType();

template <typename VType>
const double VoxelStack<VType>::m_resolution_lower_bound = 1e-2;
template <typename VType>
const double VoxelStack<VType>::m_resolution_upper_bound = 0.3;

template <typename VType>
VoxelStack<VType>::VoxelStack()
    : m_stack_size({0, 0, 0}), m_bottom_left_vertex({0, 0, 0}),
      m_top_right_vertex({0, 0, 0}), m_num_of_voxels({0, 0, 0}),
      m_total_voxels(0), m_stride_x(0), m_stride_y(0)
{
    m_resolution = m_resolution_lower_bound;
    RVS_DEBUG("Initialize voxel stack....");
    m_voxels.clear();
}

template <typename VType>
VoxelStack<VType>::VoxelStack(const std::array<double, 3> &stack_bottom_left,
                              const std::array<double, 3> &stack_top_right,
                              double resolution)
    : VoxelStack()
{
    RVS_DEBUG("Initialize voxel stack....");
    if (!_IsResolutionValid(resolution)) {
        RVS_WARN("The input resolution is {}, but required to be in range "
                 "[{}, {}]. Thus, it will be clamped. Or you can reset it.",
                 resolution, m_resolution_lower_bound,
                 m_resolution_upper_bound);
    }
    m_resolution = std::clamp(resolution, m_resolution_lower_bound,
                              m_resolution_upper_bound);

    // ensure that the vertex whose z-coordinate is smaller is assigned into
    // left bottom vertex of cuboid stack
    m_bottom_left_vertex = stack_bottom_left[2] <= stack_top_right[2]
                               ? stack_bottom_left
                               : stack_top_right;
    m_top_right_vertex = stack_top_right[2] > stack_bottom_left[2]
                             ? stack_top_right
                             : stack_bottom_left;
}

template <typename VType>
inline bool VoxelStack<VType>::_IsRangeOfStackValid()
{
    CVec3d temp_stack_bl(m_bottom_left_vertex.data());
    CVec3d temp_stack_tr(m_top_right_vertex.data());
    CVec3d temp_stack_range = (temp_stack_bl - temp_stack_tr).cwiseAbs();
    // determine whether cuboid stack is reduced to 2-dimension
    if (NearZero(temp_stack_range[0]) || NearZero(temp_stack_range[1])
        || NearZero(temp_stack_range[2])) {
        return false;
    }
    m_stack_size = std::array<double, 3>(
        {temp_stack_range.x(), temp_stack_range.y(), temp_stack_range.z()});
    return true;
}

template <typename VType>
inline bool VoxelStack<VType>::_IsResolutionValid(double resolution) const
{
    if (!(resolution >= m_resolution_lower_bound
          && resolution <= m_resolution_upper_bound)) {
        return false;
    }
    return true;
}

template <typename VType>
inline void
VoxelStack<VType>::SetStackRange(const std::array<double, 3> &stack_bottom_left,
                                 const std::array<double, 3> &stack_top_right)
{
    m_bottom_left_vertex = stack_bottom_left[2] <= stack_top_right[2]
                               ? stack_bottom_left
                               : stack_top_right;
    m_top_right_vertex = stack_top_right[2] > stack_bottom_left[2]
                             ? stack_top_right
                             : stack_bottom_left;
}

template <typename VType>
inline bool VoxelStack<VType>::HasInitializedStack() const
{
    CVec3d temp_size(m_stack_size.data());
    // herein, the stack size is utilized to indicate whether the stack
    // is succeeded in initializing
    if (NearZero(temp_size.norm())) {
        return false;
    }
    return true;
}

template <typename VType>
inline const std::array<double, 3> &VoxelStack<VType>::GetStackSize() const
{
    if (!HasInitializedStack()) {
        RVS_ERROR("Uninitialized voxel stack!");
    }
    return m_stack_size;
}

template <typename VType>
inline std::array<double, 3> VoxelStack<VType>::GetReferPoint() const
{
    if (!HasInitializedStack()) {
        RVS_ERROR("Uninitialized voxel stack!");
        return std::array<double, 3>({0.0, 0.0, 0.0});
    }
    return m_bottom_left_vertex;
}

template <typename VType>
inline void VoxelStack<VType>::SetStackResolution(double resolution)
{
    if (!_IsResolutionValid(resolution)) {
        RVS_WARN("The input resolution is {}, but required to be in range "
                 "[{}, {}]. Thus, it will be clamped. Or you can reset it.",
                 resolution, m_resolution_lower_bound,
                 m_resolution_upper_bound);
    }
    m_resolution = std::clamp(resolution, m_resolution_lower_bound,
                              m_resolution_upper_bound);
}

template <typename VType>
inline double VoxelStack<VType>::GetStackResolution() const
{
    return m_resolution;
}

template <typename VType>
inline void VoxelStack<VType>::ResetVoxelStack(const VType &ref_voxel)
{
    if (!m_voxels.empty()) {
        for (size_t i = 0; i < m_voxels.size(); i++) {
            m_voxels[i] = std::make_shared<VType>(ref_voxel);
        }
    }
    else {
        RVS_WARN("The voxel stack is empty!");
    }
}

template <typename VType>
inline bool VoxelStack<VType>::InitializeVoxelStack()
{
    if (!_IsRangeOfStackValid()) {
        RVS_ERROR("Invalid range of stack. The rectangular cuboid is required");
        return false;
    }
    if (m_resolution == m_resolution_lower_bound) {
        RVS_WARN("The resolution of stack seems to be lower bound. If this is "
                 "exactly required one, please ignore the warning!");
    }
    CVec3d temp_range(m_stack_size.data());
    temp_range /= m_resolution;
    Eigen::Vector3i temp = temp_range.cast<int>();
    m_num_of_voxels = std::array<int, 3>({temp[0], temp[1], temp[2]});
    m_stride_x = m_num_of_voxels[1] * m_num_of_voxels[2];
    m_stride_y = m_num_of_voxels[2];
    m_total_voxels = 1;
    for (const auto &v : m_num_of_voxels) {
        m_total_voxels *= v;
    }
    m_voxels.clear();
    m_voxels.resize(m_total_voxels);
    return true;
}

template <typename VType>
inline std::array<int, 3> VoxelStack<VType>::_GetVoxelIdxFromCoord(
    const std::array<double, 3> &coord) const
{
    CVec3d offset_pos;
    offset_pos.setConstant(0.5);
    CVec3d curr_pos(coord.data());
    CVec3d ref_origin_pos(m_bottom_left_vertex.data());

    /**
     * The formula that calculate the 3d index of voxel from its world
     * coordinate is as follow:
     *            coord_in - orig_ref
     * idx_3d =  ----------------------- + 0.5
     *                resolution
     *
     *The final result is : idx_3d_f = floor(idx_3d).
     *The meaning of parameters:
            @coord_in       input world coordinate of voxel
            @orig_ref       the reference point of the stack, i.e., it
                            is left bottom vertex of cuboid stack
            @resolution     sampling interval of stack
     */
    CVec3d temp_curr_idx =
        (curr_pos - ref_origin_pos) / m_resolution + offset_pos;
    Eigen::Vector3d curr_idx = temp_curr_idx.array().floor();
    return std::array<int, 3>(
        {int(curr_idx[0]), int(curr_idx[1]), int(curr_idx[2])});
}

template <typename VType>
inline std::array<double, 3> VoxelStack<VType>::_GetVoxelCoordFromIdx(
    const std::array<int, 3> &voxel_idx) const
{
    CVec3d ref_origin_pos(m_bottom_left_vertex.data());
    CVec3d curr_idx(voxel_idx[0], voxel_idx[1], voxel_idx[2]);
    CVec3d curr_pos = ref_origin_pos + curr_idx * m_resolution;
    return std::array<double, 3>({curr_pos[0], curr_pos[1], curr_pos[2]});
}

template <typename VType>
inline bool
VoxelStack<VType>::IsValidVoxel(const std::array<int, 3> &voxel_idx) const
{
    return (voxel_idx[0] >= 0 && voxel_idx[0] < m_num_of_voxels[0])
           && (voxel_idx[1] >= 0 && voxel_idx[1] < m_num_of_voxels[1])
           && (voxel_idx[2] >= 0 && voxel_idx[2] < m_num_of_voxels[2]);
}

template <typename VType>
inline bool VoxelStack<VType>::IsValidVoxel(int i_x, int i_y, int i_z) const
{
    return IsValidVoxel(std::array<int, 3>({i_x, i_y, i_z}));
}

template <typename VType>
inline int VoxelStack<VType>::_MappingIndexFrom3DTo1D(const int &i_x,
                                                      const int &i_y,
                                                      const int &i_z) const
{
    return i_x * m_stride_x + i_y * m_stride_y + i_z;
}

template <typename VType>
inline VType &VoxelStack<VType>::GetVoxel(const std::array<int, 3> &voxel_idx)
{
    if (!HasInitializedStack()) {
        RVS_ERROR("Uninitialized voxel stack!");
        return m_uninitialized_voxel;
    }
    if (!IsValidVoxel(voxel_idx)) {
        RVS_ERROR("The voxel index is out of range!");
        return m_uninitialized_voxel;
    }
    auto idx =
        _MappingIndexFrom3DTo1D(voxel_idx[0], voxel_idx[1], voxel_idx[2]);
    return *m_voxels[idx];
}

template <typename VType>
inline VType &VoxelStack<VType>::GetVoxel(int i_x, int i_y, int i_z)
{
    if (!HasInitializedStack()) {
        RVS_ERROR("Uninitialized voxel stack!");
        return m_uninitialized_voxel;
    }
    if (!IsValidVoxel(i_x, i_y, i_z)) {
        RVS_ERROR("The voxel index is out of range!");
        return m_uninitialized_voxel;
    }
    auto idx = _MappingIndexFrom3DTo1D(i_x, i_y, i_z);
    return *m_voxels[idx];
}

template <typename VType>
inline int
VoxelStack<VType>::GetNumOfVoxelsAtDim(StackDimension stack_dim) const
{
    if (!HasInitializedStack()) {
        RVS_ERROR("Uninitialized voxel stack!");
        return -1;
    }
    return m_num_of_voxels[stack_dim];
}

template <typename VType>
inline bool VoxelStack<VType>::UpdateVoxel(const std::array<int, 3> &voxel_idx,
                                           const VType &voxel)
{
    if (!HasInitializedStack()) {
        RVS_ERROR("Uninitialized voxel stack!");
        return false;
    }
    if (!IsValidVoxel(voxel_idx)) {
        RVS_ERROR("The voxel doesn't lie in voxel stack!");
        return false;
    }
    *m_voxels[_MappingIndexFrom3DTo1D(voxel_idx[0], voxel_idx[1],
                                      voxel_idx[2])] = voxel;
    return true;
}

template <typename VType>
inline bool VoxelStack<VType>::UpdateVoxel(int i_x, int i_y, int i_z,
                                           const VType &voxel)
{
    return UpdateVoxel(std::array<int, 3>({i_x, i_y, i_z}), voxel);
}

template <typename VType>
inline bool VoxelStack<VType>::IndexToCoord(const std::array<int, 3> &voxel_idx,
                                            std::array<double, 3> &coord) const
{
    coord = std::array<double, 3>({std::numeric_limits<double>::infinity(),
                                   std::numeric_limits<double>::infinity(),
                                   std::numeric_limits<double>::infinity()});
    if (!HasInitializedStack()) {
        RVS_ERROR("Uninitialized voxel stack!");
        return false;
    }
    auto is_valid = IsValidVoxel(voxel_idx);
    if (is_valid) {
        coord = _GetVoxelCoordFromIdx(voxel_idx);
    }
    return is_valid;
}

template <typename VType>
inline bool VoxelStack<VType>::IndexToCoord(int i_x, int i_y, int i_z,
                                            double &coord_x, double &coord_y,
                                            double &coord_z) const
{
    std::array<double, 3> temp_coord;
    auto is_valid =
        IndexToCoord(std::array<int, 3>({i_x, i_y, i_z}), temp_coord);
    coord_x = temp_coord[0];
    coord_y = temp_coord[1];
    coord_z = temp_coord[2];
    return is_valid;
}

template <typename VType>
inline bool VoxelStack<VType>::CoordToIndex(const std::array<double, 3> &coord,
                                            std::array<int, 3> &voxel_idx) const
{
    voxel_idx = std::array<int, 3>({-1, -1, -1});
    if (!HasInitializedStack()) {
        RVS_ERROR("Uninitialized voxel stack!");
        return false;
    }
    std::array<int, 3> temp_idx;
    temp_idx = _GetVoxelIdxFromCoord(coord);
    auto is_valid = IsValidVoxel(temp_idx);
    if (is_valid) {
        voxel_idx = temp_idx;
    }
    return is_valid;
}

template <typename VType>
inline bool VoxelStack<VType>::CoordToIndex(double coord_x, double coord_y,
                                            double coord_z, int &i_x, int &i_y,
                                            int &i_z) const
{
    std::array<int, 3> temp_idx;
    auto is_valid = CoordToIndex(
        std::array<double, 3>({coord_x, coord_y, coord_z}), temp_idx);
    i_x = temp_idx[0];
    i_y = temp_idx[1];
    i_z = temp_idx[2];
    return is_valid;
}
/// @}
} // namespace RVS