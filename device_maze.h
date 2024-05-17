class Cell
{
public:
    Cell() : leftWall(false), rightWall(false), topWall(false), bottomWall(false), value(99), isGoal(false), isStart(false) {}
    void setLeftWall(bool wall);
    void setRightWall(bool wall);
    void setTopWall(bool wall);
    void setBottomWall(bool wall);
    void setVal(int val);
    bool getLeftWall(void);
    bool getRightWall(void);
    bool getTopWall(void);
    bool getBottomWall(void);
    void setIsGoal(bool isGoal);
    void setIsStart(bool isStart);
    bool getIsGoal(void);
    bool getIsStart(void);
    int getVal(void);
    void reset(void);

private:
    bool leftWall, rightWall, topWall, bottomWall, isGoal, isStart;
    int value;
};

class Maze
{
public:
    void init(int rows, int cols);
    void setCell(int row, int col, Cell *cell);
    Cell *getCell(int row, int col);
    void shiftLeft(void);
    void shiftRight(void);
    void display(void);
    void floodFill(int startRow, int startCol, int newValue);
    void floodFillTwice();
    void setLeftWall(int row, int col, bool wall);
    void setRightWall(int row, int col, bool wall);
    void setTopWall(int row, int col, bool wall);
    void setBottomWall(int row, int col, bool wall);
    void print(void);
    void getGoalCoord(int *row, int *col);
    void getStartCoord(int *row, int *col);
    Maze *duplicate(void);

private:
    int rows,
        cols;
    Cell **maze;
};