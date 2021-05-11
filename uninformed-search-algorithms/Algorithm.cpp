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
Algorithm::Algorithm(const cv::Mat& img, const cv::Mat &bin_img)
{
    this->img = img;
    this->bin_img = bin_img;
}

/** @brief Check the point is white or not.
 * @param point (x,y) point.
 * @returns True is black, false otherwise.
*/
bool Algorithm::is_white(const cv::Point& point)
{
    if(!out_bounds(point))
    {
        // Get RGB from this pixel, (255,255,255) = white.
        cv::Vec3b color = bin_img.at<uchar>(cv::Point2i(point.x, point.y));
        if(color.val[0] == 255 or color.val[1] == 255 or color.val[2] == 255)
            return true;

        // cv::Vec3b color = bin_img.at<cv::Vec3b>(cv::Point(point.first, point.second));
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

/** @brief Will apply the corresponding move to the state.
 * @param state Current state to move.
 * @returns Adjacents from the state point.
*/
std::deque<cv::Point> Algorithm::get_adjacents(cv::Point state)
{
    // x - 1 -> left, x + 1 -> righ, y + 1 -> down, y - 1 -> up
    return { {state.x - 1, state.y}, {state.x + 1, state.y}, {state.x, state.y + 1}, {state.x, state.y - 1} };
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

/** @brief Algorithm to generate a path.
 * @param initial_state Initial state.
 * @param final_state The goal to reach.
 */
void Algorithm::breadth_first_search(cv::Point &initial_state, const cv::Point &final_state)
{
    path = nullptr;
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
 * @param initial_state Initial state.
 * @param final_state The goal to reach.
 */
void Algorithm::depth_first_search(cv::Point& initial_state, const cv::Point& final_state)
{
    path = nullptr;
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
 * @param current_state Initial state.
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
        cv::imshow("Solved", img);
        cv::line(img, start, end, cv::Scalar(10, 255, 127), 2);
        cv::waitKey(1);

        path = path->_parent;
    }
}
