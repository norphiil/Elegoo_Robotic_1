class Cell
{
public:
    Cell() : leftWall(false), rightWall(false), topWall(false), bottomWall(false), value(0) {}
    void setLeftWall(bool wall);
    void setRightWall(bool wall);
    void setTopWall(bool wall);
    void setBottomWall(bool wall);
    void setVal(int val);
    void print(void);

private:
    bool leftWall, rightWall, topWall, bottomWall;
    int value;
};

class Maze
{
public:
    void init(int rows, int cols);
    void setCell(int row, int col, Cell cell);
    Cell getCell(int row, int col);
    void print(void);

private:
    int rows, cols;
    Cell **maze;
};