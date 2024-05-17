#include "device_maze.h"
#include <Arduino.h>

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

Maze *Maze::duplicate(void)
{
    Maze *newMaze = new Maze();
    newMaze->init(rows, cols);

    for (int i = 0; i < rows; ++i)
    {
        for (int j = 0; j < cols; ++j)
        {
            // Assuming Cell has a copy constructor or a function to duplicate itself
            newMaze->maze[i][j] = this->maze[i][j];
        }
    }

    return newMaze;
}

void Maze::shiftLeft()
{
    for (int i = 0; i < this->rows; i++)
    {
        for (int j = 0; j < this->cols; j++)
        {
            if (j == this->cols - 1)
            {
                this->maze[i][j].reset();
            }
            else
            {
                this->maze[i][j] = this->maze[i][j + 1];
            }
        }
    }
}

void Maze::shiftRight()
{
    for (int i = 0; i < this->rows; i++)
    {
        for (int j = this->cols; j >= 0; j--)
        {
            if (j == 0)
            {
                this->maze[i][j].reset();
            }
            else
            {
                this->maze[i][j] = this->maze[i][j - 1];
            }
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

void Maze::display(void)
{
    for (int i = 0; i < this->rows; ++i)
    {
        // Print the top walls
        for (int j = 0; j < this->cols; ++j)
        {
            Serial.print(this->maze[i][j].getTopWall() ? "+---" : "+   ");
        }
        Serial.println("+"); // Newline at the end of the row

        // Print the left walls and cell values
        for (int j = 0; j < this->cols; ++j)
        {
            Serial.print(this->maze[i][j].getLeftWall() ? "| " : "  "); // Left wall
            if (this->maze[i][j].getIsStart())
            {
                Serial.print("S "); // Start cell
            }
            else if (maze[i][j].getIsGoal())
            {
                Serial.print("G "); // Goal cell
            }
            else
            {
                Serial.print(this->maze[i][j].getVal() < 10 ? " " : ""); // Padding for single digit numbers
                Serial.print(this->maze[i][j].getVal());                 // Empty cell
            }
        }
        Serial.println(this->maze[i][this->cols - 1].getRightWall() ? "|" : "  "); // Right wall of the last cell and newline
    }

    // Print the bottom walls
    for (int j = 0; j < this->cols; ++j)
    {
        Serial.print(this->maze[this->rows - 1][j].getBottomWall() ? "+---" : "+   ");
    }
    Serial.println("+"); // Newline at the end of the maze
}

void Maze::print(void)
{
    for (int i = 0; i < this->rows; ++i)
    {
        for (int j = 0; j < this->cols; ++j)
        {
            Serial.print("Cell(");
            Serial.print(i);
            Serial.print(", ");
            Serial.print(j);
            Serial.print("T: ");
            Serial.print(this->maze[i][j].getTopWall());
            Serial.print("B: ");
            Serial.print(this->maze[i][j].getBottomWall());
            Serial.print("L: ");
            Serial.print(this->maze[i][j].getLeftWall());
            Serial.print("R: ");
            Serial.print(this->maze[i][j].getRightWall());
            Serial.print("V: ");
            Serial.print(this->maze[i][j].getVal());
            Serial.print("G: ");
            Serial.print(this->maze[i][j].getIsGoal());
            Serial.print("S: ");
            Serial.print(this->maze[i][j].getIsStart());
            Serial.print(") |");
        }
        Serial.println();
    }
}

void Maze::floodFill(int row, int col, int newVal, int lastDir)
{
    if (this->maze[row][col].getVal() != 0 && this->maze[row][col].getVal() < newVal)
    {
        return; // If the cell is not the previous value we want to replace
    }
    // Replace the value at current cell
    this->maze[row][col].setVal(newVal);

    int cost = 1;
    // Recur for north, east, south, and west, checking for walls
    if (!this->maze[row][col].getTopWall())
    {
        if (lastDir != 1 && lastDir != 0)
            cost = 2;
        this->floodFill(row - 1, col, newVal + cost, 1); // North
    }
    if (!this->maze[row][col].getBottomWall())
    {
        if (lastDir != 2 && lastDir != 0)
            cost = 2;
        this->floodFill(row + 1, col, newVal + cost, 2); // South
    }
    if (!this->maze[row][col].getLeftWall())
    {
        if (lastDir != 3 && lastDir != 0)
            cost = 2;
        this->floodFill(row, col - 1, newVal + cost, 3); // West
    }
    if (!this->maze[row][col].getRightWall())
    {
        if (lastDir != 4 && lastDir != 0)
            cost = 2;
        this->floodFill(row, col + 1, newVal + cost, 4); // East
    }
}

void Maze::floodFillTwice()
{
    Maze *tempFirstMaze = this->duplicate();
    Maze *tempSecondMaze = this->duplicate();
    int startRow, startCol;
    this->getStartCoord(&startRow, &startCol);
    int goalRow, goalCol;
    this->getGoalCoord(&goalRow, &goalCol);
    tempFirstMaze->floodFill(startRow, startCol, 1, 0);
    tempSecondMaze->floodFill(goalRow, goalCol, 1, 0);
    for (int i = 0; i < this->rows; ++i)
    {
        for (int j = 0; j < this->cols; ++j)
        {
            int sum = tempFirstMaze->maze[i][j].getVal() + tempSecondMaze->maze[i][j].getVal();
            if (sum > 99)
            {
                sum = 99;
            }
            this->maze[i][j].setVal(sum);
        }
    }
}

void Maze::getGoalCoord(int *row, int *col)
{
    for (int i = 0; i < this->rows; ++i)
    {
        for (int j = 0; j < this->cols; ++j)
        {
            if (this->maze[i][j].getIsGoal())
            {
                *row = i;
                *col = j;
                return;
            }
        }
    }
}

void Maze::getStartCoord(int *row, int *col)
{
    for (int i = 0; i < this->rows; ++i)
    {
        for (int j = 0; j < this->cols; ++j)
        {
            if (this->maze[i][j].getIsStart())
            {
                *row = i;
                *col = j;
                return;
            }
        }
    }
}

void Maze::setLeftWall(int row, int col, bool wall)
{
    this->maze[row][col].setLeftWall(wall);
    if (col > 0)
        this->maze[row][col - 1].setRightWall(wall);
}

void Maze::setTopWall(int row, int col, bool wall)
{
    this->maze[row][col].setTopWall(wall);
    if (row > 0)
        this->maze[row - 1][col].setBottomWall(wall);
}

void Maze::setRightWall(int row, int col, bool wall)
{
    this->maze[row][col].setRightWall(wall);
    if (col + 1 < this->cols)
        this->maze[row][col + 1].setLeftWall(wall);
}

void Maze::setBottomWall(int row, int col, bool wall)
{
    this->maze[row][col].setBottomWall(wall);
    if (row + 1 < this->rows)
        this->maze[row + 1][col].setTopWall(wall);
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

void Cell::reset(void)
{
    this->leftWall = false;
    this->rightWall = false;
    this->topWall = false;
    this->bottomWall = false;
    this->isGoal = false;
    this->isStart = false;
    this->value = 0;
}

void Cell::setIsGoal(bool isGoal)
{
    this->isGoal = isGoal;
}

void Cell::setIsStart(bool isStart)
{
    this->isStart = isStart;
}

bool Cell::getIsGoal(void)
{
    return this->isGoal;
}

bool Cell::getIsStart(void)
{
    return this->isStart;
}