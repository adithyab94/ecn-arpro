#include <a_star.h>
#include <maze.h>
#include <cmath>
using namespace ecn;

// a node is a x-y position, we move from 1 each time
class Position : public Point
{
    typedef std::unique_ptr<Position> PositionPtr;

public:

    int distance = 0;
    //direction vectors to move in four directions
    std::vector<int> pose_x{-1, 0, 1, 0};
    std::vector<int> pose_y{0, 1, 0, -1};
    static Point startPoint, endPoint;

    // constructor from coordinates
    Position(int _x, int _y) : Point(_x, _y) {}

    // constructor from base ecn::Point
    Position(ecn::Point p) : Point(p.x, p.y) {}

    // constructor that also stores the distance
    Position(int _x, int _y, int dist) : Point(_x, _y)
    {
        distance = dist;
    }

    int distToParent() // Position.distance getter
    {
        return distance;
    }

    //check if current pos is corridor
    bool isCorridor(int x, int y, int &pose_x, int &pose_y)
    {
        if (pose_x && (int)maze.isFree(x + pose_x, y) + (int)maze.isFree(x, y + 1) + (int)maze.isFree(x, y - 1) == 1)
        {
            pose_y = maze.isFree(x, y + 1) ? 1 : maze.isFree(x, y - 1) ? -1 : pose_y;
            pose_x = maze.isFree(x, y + 1) || maze.isFree(x, y - 1) ? 0 : pose_x;
            return true;
        }
        else if (pose_y && (int)maze.isFree(x, y + pose_y) + (int)maze.isFree(x + 1, y) + (int)maze.isFree(x - 1, y) == 1)
        { // If it's a vertical corridor do the same checks and follow the new corridor
            pose_y = maze.isFree(x - 1, y) || maze.isFree(x + 1, y) ? 0 : pose_y;
            pose_x = maze.isFree(x-1,y) ? -1 : maze.isFree(x+1, y) ? 1 : pose_x;
            return true; //
        }
        return false;
    }
    std::vector<PositionPtr> children()
    {
        int ex = endPoint.x;
        int ey = endPoint.y;
        // this method should return  all positions reachable from this one
        std::vector<PositionPtr> generated;
        for (int k = 0; k < 4; k++)
        {
            int dist_x = pose_x[k];
            int dist_y = pose_y[k];
            int new_x = x + dist_x;
            int new_y = y + dist_y;

            if (maze.isFree(new_x, new_y))
            {
                while (isCorridor(new_x, new_y, dist_x, dist_y) && (new_x != ex || new_y != ey))
                {
                    new_x += dist_x;
                    new_y += dist_y;
                }
                generated.push_back(std::make_unique<Position>(new_x, new_y, Point(new_x, new_y).h(Point(x,y), true)));
            }
        }
        return generated;


    }

    // Point::print is not useful anymore so change to fit the algorithm
    // maze deals with the color, just tell the points
    void print(const Point &parent)
    {
        int x_start = startPoint.x;
        int y_start = startPoint.y;
        std::vector<Point> visitedCells;
        for (int k = 0; k < 4; k++)
        {
            visitedCells.clear();
            int dist_x = pose_x[k];
            int dist_y = pose_y[k];
            int new_x = x + dist_x;
            int new_y = y + dist_y;
            if (maze.isFree(new_x, new_y))
            {
                visitedCells.push_back(Point(new_x, new_y)); // Keep a record of cells we went through.
                while (isCorridor(new_x, new_y, dist_x, dist_y) && (new_x != x_start || new_y != y_start))
                {
                    new_x += dist_x;
                    new_y += dist_y;
                    visitedCells.push_back(Point(new_x, new_y)); // Keep a record of cells we went through.
                }
                if (new_x == parent.x && new_y == parent.y)
                {
                    for (const auto &p : visitedCells) // Re-trace our path
                    {
                        maze.passThrough(p.x, p.y);
                    }
                    break;
                }
            }
        }
        maze.passThrough(x, y);
    }

    //Point::show is not useful anymore so change to fit the algorith
    // online print, color depends on closed / open set
    void show(bool closed, const Point &parent)
    {
        const int b = closed ? 255 : 0, r = closed ? 0 : 255;
        int x_start = startPoint.x;
        int y_start = startPoint.y;
        std::vector<Point> visitedCells;
        for (int k = 0; k < 4; k++)
        {
            visitedCells.clear();
            int dist_x = pose_x[k];
            int dist_y = pose_y[k];
            int new_x = x + dist_x;
            int new_y = y + dist_y;

            if (maze.isFree(new_x, new_y))
            {
                visitedCells.push_back(Point(new_x, new_y)); // Store the intermediate steps
                while (isCorridor(new_x, new_y, dist_x, dist_y) && (new_x != x_start || new_y != y_start))
                {
                    new_x += dist_x;
                    new_y += dist_y;
                    visitedCells.push_back(Point(new_x, new_y)); // Store the intermediate steps
                }
                if (new_x == parent.x && new_y == parent.y)
                {
                    for (const auto &p : visitedCells) // Re-trace our path once we find the parent cell
                    {
                        maze.write(p.x, p.y, r, 0, b, false);
                    }
                    break;
                }
            }
        }
        maze.write(x, y, r, 0, b);
    }

};
Point Position::startPoint, Position::endPoint;


int main( int argc, char **argv ){
    // load file
    std::string filename = "maze.png";
    if(argc == 2)
        filename = std::string(argv[1]);

    // let Point know about this maze
    Position::maze = ecn::Maze(filename);

    // initial and goal positions as Position's
    Position start = Position::maze.start(),
             goal = Position::maze.end();

    // call A* algorithm
    ecn::Astar(start, goal);

    // save final image
    Position::maze.saveSolution("corridor");
    cv::waitKey(0);

}
