#include "device_maze.h"

//////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////      MAZE     ////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////

void Maze::init() {
  this->start = new Cell();
  this->goal = new Cell();
  this->current = this->start;
}

Heading Maze::straightToLeft(Heading heading) {
  switch (heading) {
    case NORTH:
      return WEST;
    case EAST:
      return NORTH;
    case SOUTH:
      return EAST;
    case WEST:
      return SOUTH;
    default:
      return -1;
  }
}

void Maze::update(Heading heading, bool leftWall, bool straightWall) {
  Serial.print("update heading: ");
  Serial.println(heading);
  if (!leftWall) {  // we can go there
    this->current->addNeighbour(straightToLeft(heading));
  }
  if (!straightWall) {
    this->current->addNeighbour(heading);
  }
}

void Maze::move(Heading heading) {
  // update the current to the new cell
  // for (Neighbour neighbour : current->neighbours) {
  //   if (neighbour.direction == heading) {
  //     this->current = neighbour.cell;
  //     return;
  //   }
  // }
}

void Cell::addNeighbour(Heading heading) {
  Neighbour neighbour;
  // neighbour.cell = new Cell();
  // neighbour.direction = heading;
  Serial.println(heading); 
  this->neighbours.push_back(heading);
  Serial.print("neighbour sizse: ");
  Serial.println(this->neighbours.size());
}