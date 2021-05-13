#include "Algorithm.h"


Algorithm::Algorithm() :
    path {nullptr}
{
}

size_t Algorithm::get_nodes_expanded() const
{
    return nodes_expanded;
}

size_t Algorithm::get_level() const
{
    return path->_level;
}

Algorithm::~Algorithm()
{
    for(auto& e : all_nodes) {
        delete e;
        e = nullptr;
    }
}

/** @brief Receives two images, one to analize and other to draw.
 * @param img Normal Img to be draw.
 * @param bin_img Binary Image to be analized.
 */
Algorithm::Algorithm(cv::Mat& img, cv::Mat &bin_img)
{
    this->img = img.clone();
    this->bin_img = bin_img.clone();
}

/** @brief Check the point is white or not.
 * @param point (x,y) point.
 * @returns True is black, false otherwise.
*/
bool Algorithm::is_white(const cv::Point& point)
{
    if(!out_bounds(point))
    {
        // One channel, values from 0-255.
        int color = (int)(bin_img.at<uchar>(cv::Point(point)));
        if(color == 255)
            return true;
    }

    return false;
}

/** @brief Check the point is in the image bounds.
 * @param point (x,y) point.
 * @returns True is in the bounds, false otherwise.
 */
bool Algorithm::out_bounds(const cv::Point& point)
{
    if(point.x < 0 or point.y < 0 or
       point.x >= bin_img.cols or point.y >= bin_img.rows)
    {
        return true;
    }
    return false;
}

/** @brief Will search in all white pixels to get the nearest white pixel giben a ponit.
 *  @param point Point to be compared, needed to get the min distance between white pixels and the point.
 *  @returns Nearest point to the user's point.
 *  */
cv::Point Algorithm::nearest_white_pixel(cv::Point& point)
{
    std::vector<cv::Point> white_pixels;
    std::vector<cv::Point> distances;
    double max_dist = DBL_MAX;
    cv::Point min_point;

    cv::findNonZero(bin_img, white_pixels); // Get all white pixels.
    for(auto& pixel_white : white_pixels)
    {
        double distance = cv::norm(pixel_white - point); // Euclidian distance.
        if(distance < max_dist)
        {
            max_dist = distance;
            min_point = pixel_white;
        }
    }

    return min_point;
}

/** @brief Will apply eight-connectivity.
 * @param state Current state to move.
 * @returns Adjacents from the state point.
*/
std::deque<cv::Point> Algorithm::get_adjacents(cv::Point state)
{
    // x - 1 -> left, x + 1 -> righ, y + 1 -> down, y - 1 -> up
    // x + 1, y - 1 -> top right corner.
    // x - 1, y - 1 -> top left corner.
    // x + 1, y + 1 -> bottom right corner.
    // x - 1, y + 1 -> bottom left corner.
    return { {state.x - 1, state.y}, {state.x + 1, state.y},
             {state.x, state.y + 1}, {state.x, state.y - 1},
             {state.x + 1, state.y - 1}, {state.x - 1, state.y - 1},
             {state.x + 1, state.y + 1}, {state.x -1, state.y + 1} };
}

/** @brief: Will expand the current node, generating the next nodes.
 * @param current_state Current node that holds the current state.
 * @returns A deque of the expanded nodes based on current state. */
std::deque<Node *> Algorithm::expand_node(Node *current_state)
{
    ++nodes_expanded;
    std::deque<cv::Point> aux = get_adjacents(current_state->_point);
    std::deque<Node *> adjacents;

    for(const auto& point : aux)
        adjacents.push_back(new Node(point, current_state, current_state->_level + 1));
    return adjacents;
}

double Algorithm::heuristic(cv::Point start, cv::Point end)
{
    return abs(end.x - start.x) + abs(end.y - start.y);
}

/** @brief Algorithm to generate a path.
 * @param initial_state Initial state to start.
 * @param final_state The goal to reach.
 */
void Algorithm::breadth_first_search(cv::Point &initial_state, const cv::Point &final_state)
{
    Node *root = new Node(initial_state, nullptr, 0);
    std::deque<Node *> my_deque;
    std::set< cv::Point, comparePoints > visited;
    all_nodes.push_back(root);
    my_deque.push_back(root);

    while(!my_deque.empty())
    {
        Node *current_state = my_deque.front();
        my_deque.pop_front();
        visited.insert(current_state->_point);

        if(current_state->_point == final_state)
        {
            path = current_state;
            break;
        }

        std::deque<Node *> possible_states = expand_node(current_state);
        all_nodes.insert(all_nodes.end(), possible_states.begin(), possible_states.end());

        for(const auto& state: possible_states)
        {
            // If the state hasn't been visited yet.
            if(visited.find(state->_point) == visited.end() and !out_bounds(state->_point) and is_white(state->_point))
            {
                my_deque.push_back(std::move(state));
                visited.insert(state->_point);
            }
        }
    }
}

/** @brief Algorithm to generate a path.
 * @param initial_state Initial state to start.
 * @param final_state The goal to reach.
 */
void Algorithm::depth_first_search(cv::Point& initial_state, const cv::Point& final_state)
{
    Node *root = new Node(initial_state, nullptr, 0);
    all_nodes.push_back(root);
    std::stack<Node *> my_stack;
    std::set< cv::Point, comparePoints > visited;
    my_stack.push(root);

    while(!my_stack.empty())
    {
        Node* current_state = my_stack.top();
        my_stack.pop();
        visited.insert(current_state->_point);

        if(current_state->_point == final_state)
        {
            path = current_state;
            break;
        }

        std::deque<Node *> possible_paths = expand_node(current_state);
        all_nodes.insert(all_nodes.end(), possible_paths.begin(), possible_paths.end());

        for(auto it = possible_paths.rbegin(); it != possible_paths.rend(); ++it)
        {
            if(visited.find((*it)->_point ) == visited.end() and !out_bounds((*it)->_point) and is_white((*it)->_point))
            {
                my_stack.push(std::move(*it));
                visited.insert((*it)->_point);
            }
        }
    }
}

/** @brie: Used with iterative deeping search to check each level based on a max depth,
 * will check or expand the corresponding states.
 * @param current_state State to be expanded or checked.
 * @param final_state The goal to reach.
 * @param visited All the states that are already visited, to improve performance.
 * @param max_depth The depth limit to search.
 * @returns True if the final state is reached, otherwise false.
 */
bool Algorithm::depth_limited_search(Node* current_state, const cv::Point& final_state, std::set< cv::Point, comparePoints >& visited, size_t max_depth)
{
    if(current_state->_point == final_state) {
        // Save the state to trace back the moves.
        path = current_state;
        return true;
    }
    if(max_depth == 0)
        {path = current_state; return false;}

    visited.insert(current_state->_point);

    std::deque<Node* > possible_states;

//    Only expand the nodes that aren't at level 0.
    if(max_depth != 0) {
        possible_states = expand_node(current_state);
        all_nodes.insert(all_nodes.end(), possible_states.begin(), possible_states.end());
    }

    for(auto it = possible_states.rbegin(); it != possible_states.rend(); ++it) {
        if(visited.find((*it)->_point) == visited.end() and !out_bounds((*it)->_point) and is_white((*it)->_point))
        {
            if(depth_limited_search(*it, final_state, visited, max_depth - 1))
                return true;
        }
    }

    return false;
}

/** @brief Algorithm to generate the moves to follow to solve the game.
 * @param initial_state Initial state to start.
 * @param final_state The goal to reach.
 * @param max_depth The depth limit to search.
 */
void Algorithm::iterative_deepening_search(cv::Point& initial_state, const cv::Point& final_state, size_t max_depth)
{
    path = nullptr;
    Node *root = new Node(initial_state, nullptr, 0);
    Node* current_state = root;
    all_nodes.push_back(root);

    for(size_t depth = 0; depth <= max_depth; ++depth)
    {
        qDebug() << depth << '\n';
        std::set< cv::Point, comparePoints > visited;
        visited.insert(current_state->_point);
        if(depth_limited_search(current_state, final_state, visited, depth))
        {
            return;
        }
    }
}

/** @brief Algorithm to find the shortest path using heuristic. (primero el mejor, Greedy Best-First Search).
  * @param current_state Initial state.
  * @param final_state The goal to reach.
  * */
void Algorithm::best_first_search(cv::Point &initial_state, const cv::Point &final_state)
{
    double distance = 0.0;
    Node *root = new Node(initial_state, nullptr, 0);
    typedef std::pair<Node*, double> pair_node_dist;

    // Our root has distance 0.
    pair_node_dist node_dist(root, 0);
    // Create a min heap.
    std::priority_queue< pair_node_dist,
                         std::vector<pair_node_dist>,
                         CompareDistance > min_heap;

    std::set< cv::Point, comparePoints > visited;
    all_nodes.push_back(root);
    min_heap.push(node_dist);

    while(!min_heap.empty())
    {
        pair_node_dist current_state = min_heap.top();
        min_heap.pop();
        visited.insert(current_state.first->_point);

        if(current_state.first->_point == final_state) {
            path = current_state.first;
            break;
        }

        std::deque<Node *> possible_states = expand_node(current_state.first);
        all_nodes.insert(all_nodes.end(), possible_states.begin(), possible_states.end());
        for(const auto& state : possible_states)
        {
            // If the state hasn't been visited yet.
            if(visited.find(state->_point) == visited.end() and !out_bounds(state->_point) and is_white(state->_point))
            {
                // Manhattan distance between start and end.
                distance = heuristic(state->_point, final_state);
                min_heap.push(pair_node_dist(std::move(state), distance));
                visited.insert(state->_point);
            }
        }
    }
}

/// Check if path exist.
bool Algorithm::path_exist()
{
    return path != nullptr ? true : false;
}

/// Draw path on the pic.
void Algorithm::draw_path()
{
    while(path->_parent != nullptr)
    {
        cv::Point start(path->_point);
        cv::Point end(path->_point);
        cv::imshow("Original", img);
        cv::line(img, start, end, cv::Scalar(10, 255, 127), 2);
        cv::waitKey(1);

        path = path->_parent;
    }
}
