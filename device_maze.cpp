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

void Maze::print()
{
    for (int i = 0; i < this->rows; i++)
    {
        for (int j = 0; j < this->cols; j++)
        {
            this->maze[i][j].print();
        }
    }
}

void Maze::setCell(int row, int col, Cell cell)
{
    maze[row][col] = cell;
}

Cell Maze::getCell(int row, int col)
{
    return maze[row][col];
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

void Cell::print()
{
    // Serial.print("L:");
    // Serial.print(leftWall);
    // Serial.print(" R:");
    // Serial.print(rightWall);
    // Serial.print(" T:");
    // Serial.print(topWall);
    // Serial.print(" B:");
    // Serial.print(bottomWall);
    // Serial.print(" V:");
    // Serial.println(value);
}