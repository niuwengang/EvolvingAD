#include "../include/lsq_registration.hpp"
#include "../include/impl/lsq_registration_impl.hpp"

template class fast_gicp::LsqRegistration<pcl::PointXYZ, pcl::PointXYZ>;
template class fast_gicp::LsqRegistration<pcl::PointXYZI, pcl::PointXYZI>;
template class fast_gicp::LsqRegistration<pcl::PointNormal, pcl::PointNormal>;
