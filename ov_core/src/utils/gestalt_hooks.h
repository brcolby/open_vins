#pragma once

// Optional observation hooks for an embedding application.
// Ordinary ROS and standalone builds require no Gestalt runtime.
#ifdef GESTALT_DIAGNOSTIC_HOOKS
extern "C" void gestalt_ov_init_branch(int);
extern "C" void gestalt_ov_stats(double,int,int,int,int,int,int,int,double,double,double,double,double,double);
extern "C" void gestalt_vio_visual_update();
#else
inline void gestalt_ov_init_branch(int) {}
inline void gestalt_ov_stats(double,int,int,int,int,int,int,int,double,double,double,double,double,double) {}
inline void gestalt_vio_visual_update() {}
#endif
