#include <a_star.h>
#include <maze.h>
#include <cmath>
using namespace std;
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

    // constructor from coordinates
    Position(int _x, int _y) : Point(_x, _y) {}

    // constructor from base ecn::Point
    Position(ecn::Point p) : Point(p.x, p.y) {}

    Position(int _x, int _y, int dist) : Point(_x, _y) {
        distance = dist;
    }

    int distToParent() // Position.distance getter
    {
        return distance;
    }

    //check if current pos is corridor
    bool isCorridor(int x, int y){
        int counter = 0;
        int new_x;
        int new_y;
        bool isCorr = false;
        for (int i = 0; i < 4; i++)
        {
            new_x = x + pose_x[i];
            new_y = y + pose_y[i];
            if(maze.isFree(new_x,new_y)){
                counter++;
            }
        }
        if (counter == 2)
            isCorr = true;
         return isCorr;
    }

    std::vector<PositionPtr> children()
    {
        // this method should return  all positions reachable from this one
        std::vector<PositionPtr> generated;
        int new_x;
        int new_y;
        for (int i = 0; i < 4; i++)
        {
            new_x = x + pose_x[i];
            new_y = y + pose_y[i];
            if (maze.isFree(new_x , new_y))
            {
                while(isCorridor(new_x, new_y)){
                    if(maze.isFree(new_x + pose_x[i], new_y +pose_y[i])){
                        new_x += pose_x[i];
                        new_y += pose_y[i];
                    }else{
                        break;
                    }
                }
                generated.push_back(std::make_unique<Position>(new_x, new_y, Point(new_x, new_y).h(Point(x,y), true)));
            }
        }
        return generated;
    }

};


int main( int argc, char **argv )
{
    // load file
    std::string filename = "maze.png";
    if(argc == 2)
        filename = std::string(argv[1]);

    // let Point know about this maze
    Position::maze.load(filename);

    // initial and goal positions as Position's
    Position start = Position::maze.start(),
             goal = Position::maze.end();

    // call A* algorithm
    ecn::Astar(start, goal);

    // save final image
    Position::maze.saveSolution("line");
    cv::waitKey(0);

}
