#include "device_maze.h"

//////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////      MAZE     ////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////

void Maze::init(int rows, int cols)
{
    this->rows = rows;
    this->cols = cols;
    this->maze = new Cell *[rows];
    for (int i = 0; i < rows; ++i)
    {
        this->maze[i] = new Cell[cols];
    }
}

void Maze::shiftLeft()
{
    for (int i = 0; i < this->rows; i++)
    {
        for (int j = 0; j < this->cols - 1; j++)
        {
            this->maze[i][j] = this->maze[i][j + 1];
        }
    }
}

void Maze::shiftRight()
{
    for (int i = 0; i < this->rows; i++)
    {
        for (int j = this->cols - 1; j > 0; j--)
        {
            this->maze[i][j] = this->maze[i][j - 1];
        }
    }
}

void Maze::setCell(int row, int col, Cell *cell)
{
    maze[row][col] = *cell;
}

Cell *Maze::getCell(int row, int col)
{
    return &maze[row][col];
}

void Cell::setLeftWall(bool wall)
{
    this->leftWall = wall;
}

void Cell::setRightWall(bool wall)
{
    this->rightWall = wall;
}

void Cell::setTopWall(bool wall)
{
    this->topWall = wall;
}

void Cell::setBottomWall(bool wall)
{
    this->bottomWall = wall;
}

void Cell::setVal(int val)
{
    this->value = val;
}

bool Cell::getBottomWall(void)
{
    return this->bottomWall;
}

bool Cell::getLeftWall(void)
{
    return this->leftWall;
}

bool Cell::getRightWall(void)
{
    return this->rightWall;
}

bool Cell::getTopWall(void)
{
    return this->topWall;
}

int Cell::getVal(void)
{
    return this->value;
}
