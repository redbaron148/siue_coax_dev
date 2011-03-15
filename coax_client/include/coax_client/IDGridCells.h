#ifndef ID_GRID_CELLS_H
#define ID_GRID_CELLS_H

/**
 *  @Author Aaron Parker
 *  @File   IDGridCells.h
 *
 *  @Description    
 */

//#include <ros/ros.h>
#include <nav_msgs/GridCells.h>

namespace coax_client
{
class IDGridCells : nav_msgs::GridCells
{
public:
    IDGridCells(){}
    ~IDGridCells(){}
    
protected:
    std::vector<uint8_t> ids;
};
}

#endif
