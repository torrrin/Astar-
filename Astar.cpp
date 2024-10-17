#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <unordered_map>
#include <algorithm>
#include <utility>

using namespace std;

// 默认只能上下左右移动，不可以斜着走
//  节点结构，存储节点坐标和计算的f, g, h值

// 节点模块
struct Node
{
	int x, y;		// 横纵坐标
	double f, g, h; // g是代价函数，指的是已经走过的路程；h是启发函数，指该节点到终点的欧氏距离；f=g+h，决定优先队列的排序
	Node *parent;	// 父节点代表到达该节点的节点

	Node(int x, int y, double g, double h, Node *parent = nullptr) // 构造函数
		: x(x), y(y), g(g), h(h), f(g + h), parent(parent)
	{
	}
	// 记住只重载了<
	bool operator<(const Node &other) const
	{
		return f > other.f; // 优先队列中较小的f值具有更高的优先级
	}
};

// 计算两节点之间的欧氏距离
double euclideanDistance(int x1, int y1, int x2, int y2)
{
	return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

// 检查一个节点是否在网格范围内
bool isInBounds(int x, int y, const vector<vector<int>> &grid)
{ // 利用二维数组表示地图
	return x >= 0 && x < grid.size() && y >= 0 && y < grid[0].size();
}

// 检查一个节点是否可通行，1代表障碍物，0代表可通行道路
bool isWalkable(int x, int y, const vector<vector<int>> &grid)
{
	return grid[x][y] == 0;
}

void printResult(Node *node, const vector<vector<int>> &grid)
{
	// 创建一个标记了路径的地图副本
	vector<vector<char>> map(grid.size(), vector<char>(grid[0].size(), ' '));

	// 填充障碍物
	for (int i = 0; i < grid.size(); ++i)
	{
		for (int j = 0; j < grid[0].size(); ++j)
		{
			if (grid[i][j] == 1)
			{
				map[i][j] = '#'; // 使用 # 表示障碍物
			}
		}
	}

	// 反向追踪路径并标记在地图上
	while (node != nullptr)
	{
		map[node->x][node->y] = '*'; // 使用 * 表示路径
		node = node->parent;
	}

	// 打印地图
	for (int i = 0; i < map.size(); ++i)
	{
		for (int j = 0; j < map[0].size(); ++j)
		{
			cout << map[i][j] << ' ';
		}
		cout << endl;
	}
}

// A*算法实现，输入地图以及起点终点信息
void aStar(const vector<vector<int>> &grid, pair<int, int> start, pair<int, int> goal)
{
	// 定义四个方向：上、下、左、右，目前不能斜着走
	vector<pair<int, int>> directions = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}, {-1, -1}, {1, -1}, {1, 1}, {-1, 1}};

	priority_queue<Node> openList;		 // 存放节点的优先队列，按照f排序
	unordered_map<int, Node *> allNodes; // 运用哈希表更好地管理节点

	// 起点
	Node *startNode = new Node(start.first, start.second, 0, euclideanDistance(start.first, start.second, goal.first, goal.second));
	openList.push(*startNode);
	allNodes[start.first * grid[0].size() + start.second] = startNode;

	// 使用unordered_map来跟踪访问过的节点
	unordered_map<int, bool> closedList;

	while (!openList.empty())
	{
		Node current = openList.top();
		openList.pop();

		// 如果找到目标节点，输出路径
		if (current.x == goal.first && current.y == goal.second)
		{
			cout << "*代表路线，#代表障碍物" << endl
				 << "Path found: " << endl;
			printResult(allNodes[current.x * grid[0].size() + current.y], grid);
			cout << endl
				 << "起点为(" << start.first << "," << start.second << ")" << "  终点为( " << goal.first << "," << goal.second << ")" << endl;
			cout << "代价为：" << current.g << endl;
			return;
		}
		closedList[current.x * grid[0].size() + current.y] = true;

		// 遍历邻居节点
		for (const auto &dir : directions)
		{
			int newX = current.x + dir.first;
			int newY = current.y + dir.second;

			if (isInBounds(newX, newY, grid) && isWalkable(newX, newY, grid) &&
				!closedList[newX * grid[0].size() + newY])
			{

				double gNew = current.g + euclideanDistance(current.x, current.y, newX, newY);
				double hNew = euclideanDistance(newX, newY, goal.first, goal.second);

				int nodeKey = newX * grid[0].size() + newY;

				if (allNodes.find(nodeKey) == allNodes.end() || gNew < allNodes[nodeKey]->g)
				{
					Node *neighbor = new Node(newX, newY, gNew, hNew, allNodes[current.x * grid[0].size() + current.y]);
					openList.push(*neighbor);
					allNodes[nodeKey] = neighbor;
				}
			}
		}
	}

	cout << "没有路线！" << endl;
}

int main()
{
	// 0表示通行，1表示障碍物
	vector<vector<int>> grid0 = {
		{0, 0, 0, 0, 1},
		{0, 1, 0, 0, 1},
		{0, 1, 0, 0, 0},
		{0, 0, 0, 1, 0},
		{1, 1, 0, 0, 0}};
	// 默认起点和终点
	pair<int, int> start0 = {0, 0};
	pair<int, int> goal0 = {4, 4};

	vector<vector<int>> grid1 = {
		{0, 0, 0, 1, 0, 0},
		{0, 1, 0, 1, 0, 0},
		{0, 0, 0, 0, 1, 0},
		{1, 1, 1, 0, 1, 0},
		{0, 0, 0, 0, 0, 0},
		{0, 1, 1, 1, 1, 0}};

	// 起点 (0, 0)，终点 (5, 5)
	pair<int, int> start1 = {0, 0};
	pair<int, int> goal1 = {5, 5};

	vector<vector<int>> grid2 = {
		{0, 0, 0, 0, 1, 0, 0, 0},
		{1, 1, 0, 1, 1, 1, 0, 0},
		{0, 0, 0, 0, 0, 1, 0, 1},
		{0, 1, 1, 1, 0, 0, 0, 0}};

	// 起点 (3, 0)，终点 (0, 7)
	pair<int, int> start2 = {3, 0};
	pair<int, int> goal2 = {0, 7};

	vector<vector<int>> grid3 = {
		{0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0},
		{0, 1, 1, 0, 1, 0, 1, 1, 0, 1, 0, 1, 0},
		{0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0},
		{1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1, 0},
		{0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0},
		{0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0},
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		{0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0},
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		{1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1},
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1},
		{0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};

	// 起点 (0, 0)，终点 (12, 12)
	pair<int, int> start3 = {0, 0};
	pair<int, int> goal3 = {12, 12};

	vector<vector<int>> grid4 = {
		{0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1},
		{0, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1},
		{0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1},
		{0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1},
		{0, 1, 1, 0, 0, 0, 0, 0, 0, 1, 0, 1, 1},
		{0, 1, 1, 0, 1, 1, 1, 1, 0, 1, 0, 1, 1},
		{0, 1, 1, 0, 0, 1, 0, 1, 0, 1, 0, 1, 1},
		{0, 1, 1, 1, 1, 0, 1, 1, 0, 1, 0, 1, 1},
		{0, 1, 1, 1, 1, 0, 1, 1, 0, 1, 0, 1, 1},
		{0, 1, 1, 1, 0, 1, 1, 1, 0, 0, 0, 1, 1},
		{0, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1},
		{0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
		{0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}};

	// 起点 (0, 0)，终点 (12, 12)
	pair<int, int> start4 = {0, 6};
	pair<int, int> goal4 = {6, 6};

	cout << "请选择地图0/1/2/3/4:";
	int mode;
	cin >> mode;
	if (mode == 0)
	{
		aStar(grid0, start0, goal0);
	}
	else if (mode == 1)
	{
		aStar(grid1, start1, goal1);
	}
	else if (mode == 2)
	{
		aStar(grid2, start2, goal2);
	}
	else if (mode == 3)
	{
		aStar(grid3, start3, goal3);
	}
	else if (mode == 4)
	{
		aStar(grid4, start4, goal4);
	}
	else
	{
		cout << "不存在该地图!";
	}

	return 0;
}
