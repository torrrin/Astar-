#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <unordered_map>
#include <algorithm>
#include <utility>

using namespace std;

// Ĭ��ֻ�����������ƶ���������б����
//  �ڵ�ṹ���洢�ڵ�����ͼ����f, g, hֵ

// �ڵ�ģ��
struct Node
{
	int x, y;		// ��������
	double f, g, h; // g�Ǵ��ۺ�����ָ�����Ѿ��߹���·�̣�h������������ָ�ýڵ㵽�յ��ŷ�Ͼ��룻f=g+h���������ȶ��е�����
	Node *parent;	// ���ڵ������ýڵ�Ľڵ�

	Node(int x, int y, double g, double h, Node *parent = nullptr) // ���캯��
		: x(x), y(y), g(g), h(h), f(g + h), parent(parent)
	{
	}
	// ��סֻ������<
	bool operator<(const Node &other) const
	{
		return f > other.f; // ���ȶ����н�С��fֵ���и��ߵ����ȼ�
	}
};

// �������ڵ�֮���ŷ�Ͼ���
double euclideanDistance(int x1, int y1, int x2, int y2)
{
	return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

// ���һ���ڵ��Ƿ�������Χ��
bool isInBounds(int x, int y, const vector<vector<int>> &grid)
{ // ���ö�ά�����ʾ��ͼ
	return x >= 0 && x < grid.size() && y >= 0 && y < grid[0].size();
}

// ���һ���ڵ��Ƿ��ͨ�У�1�����ϰ��0�����ͨ�е�·
bool isWalkable(int x, int y, const vector<vector<int>> &grid)
{
	return grid[x][y] == 0;
}

void printResult(Node *node, const vector<vector<int>> &grid)
{
	// ����һ�������·���ĵ�ͼ����
	vector<vector<char>> map(grid.size(), vector<char>(grid[0].size(), ' '));

	// ����ϰ���
	for (int i = 0; i < grid.size(); ++i)
	{
		for (int j = 0; j < grid[0].size(); ++j)
		{
			if (grid[i][j] == 1)
			{
				map[i][j] = '#'; // ʹ�� # ��ʾ�ϰ���
			}
		}
	}

	// ����׷��·��������ڵ�ͼ��
	while (node != nullptr)
	{
		map[node->x][node->y] = '*'; // ʹ�� * ��ʾ·��
		node = node->parent;
	}

	// ��ӡ��ͼ
	for (int i = 0; i < map.size(); ++i)
	{
		for (int j = 0; j < map[0].size(); ++j)
		{
			cout << map[i][j] << ' ';
		}
		cout << endl;
	}
}

// A*�㷨ʵ�֣������ͼ�Լ�����յ���Ϣ
void aStar(const vector<vector<int>> &grid, pair<int, int> start, pair<int, int> goal)
{
	// �����ĸ������ϡ��¡����ң�Ŀǰ����б����
	vector<pair<int, int>> directions = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}, {-1, -1}, {1, -1}, {1, 1}, {-1, 1}};

	priority_queue<Node> openList;		 // ��Žڵ�����ȶ��У�����f����
	unordered_map<int, Node *> allNodes; // ���ù�ϣ����õع���ڵ�

	// ���
	Node *startNode = new Node(start.first, start.second, 0, euclideanDistance(start.first, start.second, goal.first, goal.second));
	openList.push(*startNode);
	allNodes[start.first * grid[0].size() + start.second] = startNode;

	// ʹ��unordered_map�����ٷ��ʹ��Ľڵ�
	unordered_map<int, bool> closedList;

	while (!openList.empty())
	{
		Node current = openList.top();
		openList.pop();

		// ����ҵ�Ŀ��ڵ㣬���·��
		if (current.x == goal.first && current.y == goal.second)
		{
			cout << "*����·�ߣ�#�����ϰ���" << endl
				 << "Path found: " << endl;
			printResult(allNodes[current.x * grid[0].size() + current.y], grid);
			cout << endl
				 << "���Ϊ(" << start.first << "," << start.second << ")" << "  �յ�Ϊ( " << goal.first << "," << goal.second << ")" << endl;
			cout << "����Ϊ��" << current.g << endl;
			return;
		}
		closedList[current.x * grid[0].size() + current.y] = true;

		// �����ھӽڵ�
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

	cout << "û��·�ߣ�" << endl;
}

int main()
{
	// 0��ʾͨ�У�1��ʾ�ϰ���
	vector<vector<int>> grid0 = {
		{0, 0, 0, 0, 1},
		{0, 1, 0, 0, 1},
		{0, 1, 0, 0, 0},
		{0, 0, 0, 1, 0},
		{1, 1, 0, 0, 0}};
	// Ĭ�������յ�
	pair<int, int> start0 = {0, 0};
	pair<int, int> goal0 = {4, 4};

	vector<vector<int>> grid1 = {
		{0, 0, 0, 1, 0, 0},
		{0, 1, 0, 1, 0, 0},
		{0, 0, 0, 0, 1, 0},
		{1, 1, 1, 0, 1, 0},
		{0, 0, 0, 0, 0, 0},
		{0, 1, 1, 1, 1, 0}};

	// ��� (0, 0)���յ� (5, 5)
	pair<int, int> start1 = {0, 0};
	pair<int, int> goal1 = {5, 5};

	vector<vector<int>> grid2 = {
		{0, 0, 0, 0, 1, 0, 0, 0},
		{1, 1, 0, 1, 1, 1, 0, 0},
		{0, 0, 0, 0, 0, 1, 0, 1},
		{0, 1, 1, 1, 0, 0, 0, 0}};

	// ��� (3, 0)���յ� (0, 7)
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

	// ��� (0, 0)���յ� (12, 12)
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

	// ��� (0, 0)���յ� (12, 12)
	pair<int, int> start4 = {0, 6};
	pair<int, int> goal4 = {6, 6};

	cout << "��ѡ���ͼ0/1/2/3/4:";
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
		cout << "�����ڸõ�ͼ!";
	}

	return 0;
}
