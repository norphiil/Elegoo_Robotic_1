#include <Arduino.h>
#include <Vector.h>
enum Heading {
  NORTH,
  EAST,
  SOUTH,
  WEST
};

class Cell; // Forward declaration

struct Neighbour {
  Cell *cell;
  Heading direction;
};

class Cell {
public:
  Cell::Cell() : neighbours() {}

  Vector<Heading> neighbours;
  uint8_t value;
  void addNeighbour(Heading heading);
};

class Maze {
  public:
  void init();
  void move(Heading heading);
  void update(Heading heading, bool leftWall, bool straightWall);
  Heading straightToLeft(Heading heading);
  Cell* start;
  Cell* goal;
  Cell* current;
};
