from tkinter import *
from functools import partial
from time import sleep


def center_gui(root):
    windowWidth = 700
    windowHeight = 750
    positionRight = int(root.winfo_screenwidth() / 2 - windowWidth / 2)
    positionDown = int(root.winfo_screenheight() / 2 - windowHeight / 2)
    root.geometry(f"{windowWidth}x{windowHeight}+{positionRight}+{positionDown}")


class App:
    GRID_SIZE = 9
    CELL_SIZE = 140

    def __init__(self, master):
        self.master = master
        master.wm_title("A* Algorithm")
        master.resizable(False, False)

        self.cells = []
        self.start = None
        self.goal = None
        self.obstacles = set()
        self.mode = 0
        self.is_drawing = False

        # Create grid of Labels with larger size
        for i in range(self.GRID_SIZE):
            self.cells.append([])
            for j in range(self.GRID_SIZE):
                cell = Label(master, width=self.CELL_SIZE // 10, height=self.CELL_SIZE // 15,
                             font=('Arial', 7, 'bold'),
                             bg="white", relief="raised", bd=2)
                cell.bind('<Button-1>', partial(self.on_click, i, j))
                cell.bind('<B1-Motion>', partial(self.on_drag, i, j))
                cell.bind('<ButtonRelease-1>', self.stop_drawing)
                self.cells[i].append(cell)
                self.cells[i][j].grid(row=i, column=j)

        center_gui(master)

    def on_click(self, row, column, event):
        if self.mode == 0:
            self.start = (row, column)
            self.mode = 1
            self.cells[row][column].configure(bg='#4CAF50')
            print(f"Start set at ({row}, {column})")
        elif self.mode == 1:
            self.goal = (row, column)
            self.mode = 2
            self.cells[row][column].configure(bg='#F44336')
            print(f"Goal set at ({row}, {column})")
        elif self.mode == 2:
            self.mode = 3
            print("Obstacle mode activated - drag to draw obstacles")
        # Once in mode 3 (obstacle mode), clicking does nothing - only dragging adds obstacles
        self.is_drawing = True
        if self.mode == 3:
            self.on_drag(row, column, event)

    def on_drag(self, row, column, event):

        """
        This method is called when the user drags the mouse over cells.
        """
        if self.mode == 3 and self.is_drawing:
            # Don't allow obstacles at start or goal positions
            if (row, column) != self.start and (row, column) != self.goal:
                # Add obstacle if it doesn't already exist
                if (row, column) not in self.obstacles:
                    self.obstacles.add((row, column))
                    # Color it dark gray to show it's an obstacle
                    self.cells[row][column].configure(bg='#424242')

    def stop_drawing(self, event):
        self.is_drawing = False

    # calcualte Manhattan distance as heuristic
    def heuristic(self, node1, node2):
        return abs(node1[0] - node2[0]) + abs(node1[1] - node2[1])

    def find_neighbors(self, current):
        neighbors = []
        row, col = current

        # Cardinal directions only (no diagonal through walls)
        # Up, Down, Right, Left
        directions = [(0, 1), (0, -1), (1, 0), (-1, 0)]

        for dr, dc in directions:
            new_row, new_col = row + dr, col + dc
            if 0 <= new_row < self.GRID_SIZE and 0 <= new_col < self.GRID_SIZE:
                neighbor = (new_row, new_col)
                if neighbor not in self.obstacles:
                    neighbors.append(neighbor)
        return neighbors

    def reconstruct_path(self, cameFrom, current):

        """
        This method traces back the shortest path from the goal to the start.
        It uses the cameFrom dictionary which was built during A* algorithm execution.
        """

        total_path = [current]
        # Keep following the cameFrom chain until we reach the start
        while current in cameFrom and cameFrom[current] is not None:
            current = cameFrom[current]
            total_path.append(current)
        # Reverse to get path from start to goal (instead of goal to start)
        total_path.reverse()
        return total_path

    def a_star_algorithm(self, start, goal):
        """
        A* (A-Star) pathfinding algorithm - finds the shortest path from start to goal.

        Key concepts:
        - open_set: Nodes we can explore next
        - g_score: Actual distance from start to each node
        - f_score: g_score + heuristic (estimated total distance)
        - came_from: Tracks which node each cell came from (for path reconstruction)

        The algorithm picks the node with lowest f_score from open_set and explores its neighbors.
        If a better path to a neighbor is found, we update it and add to open_set.
        """
        if start is None or goal is None:
            print("Start or goal not set!")
            return

        # Initialize: nodes we can explore next
        open_set = [start]

        # Start with all infinity, then set start to 0
        g_score = [[float('inf')] * self.GRID_SIZE for _ in range(self.GRID_SIZE)]

        # f_score[row][col] = g_score + heuristic (estimated total cost)
        # Used to decide which node to explore next
        f_score = [[float('inf')] * self.GRID_SIZE for _ in range(self.GRID_SIZE)]

        # Dictionary to store: came_from[node] = previous_node (for path reconstruction)
        came_from = {}

        # Initialize start node
        g_score[start[0]][start[1]] = 0
        f_score[start[0]][start[1]] = self.heuristic(start, goal)

        # Main loop: continue while there are nodes to explore
        while open_set:
            self.master.update()  # Update GUI to show animation
            sleep(0.02)  # Slow down animation for visibility

            # Sort open_set by f_score and pick the one with lowest cost
            open_set = sorted(open_set, key=lambda node: f_score[node[0]][node[1]])
            current = open_set[0]

            # If we reached the goal, reconstruct and visualize the path
            if current == goal:
                path = self.reconstruct_path(came_from, current)
                self.visualize_path(path)
                return

            # Remove current node from open_set (we're processing it now)
            open_set.remove(current)

            # Get all neighboring cells (up, down, left, right)
            neighbors = self.find_neighbors(current)

            # Check each neighbor
            for neighbor in neighbors:
                # Calculate distance: current distance + 1 (cost to neighbor)
                tentative_gScore = g_score[current[0]][current[1]] + 1

                # If we found a better path to this neighbor
                if tentative_gScore < g_score[neighbor[0]][neighbor[1]]:
                    # Record that we came from 'current' to reach 'neighbor'
                    came_from[neighbor] = current

                    # Update the actual distance to this neighbor
                    g_score[neighbor[0]][neighbor[1]] = tentative_gScore

                    # Update the estimated total distance (g + heuristic)
                    f_score[neighbor[0]][neighbor[1]] = g_score[neighbor[0]][neighbor[1]] + self.heuristic(neighbor,
                                                                                                           goal)

                    # If this neighbor isn't in open_set yet, add it for future exploration
                    if neighbor not in open_set:
                        # Color it light blue to show it's been explored
                        if neighbor != start and neighbor != goal:
                            self.cells[neighbor[0]][neighbor[1]].configure(bg='#64B5F6')
                        open_set.append(neighbor)

        # If we exit the loop and haven't found the goal, no path exists
        print("No path found!")

    def visualize_path(self, path):
        for i, node in enumerate(path):
            row, col = node
            if node == self.start:
                self.cells[row][col].configure(bg='#4CAF50')
            elif node == self.goal:
                self.cells[row][col].configure(bg='#F44336')
            else:
                self.cells[row][col].configure(bg='#FFEB3B')
            if i < len(path) - 1:
                self.master.update()
                sleep(0.05)

    def find_path(self, event):
        # Allow pathfinding from mode 2 or mode 3
        if self.mode == 2 or self.mode == 3:
            print("Starting A* algorithm...")
            self.a_star_algorithm(self.start, self.goal)
            self.mode = 4
        else:
            print("Please set start point, goal point, and draw obstacles first")

    def reset(self, event):
        self.start = None
        self.goal = None
        self.obstacles = set()
        self.mode = 0

        for i in range(self.GRID_SIZE):
            for j in range(self.GRID_SIZE):
                self.cells[i][j].configure(bg='white')

        print("Reset complete")


if __name__ == '__main__':
    root = Tk()
    app = App(root)

    root.bind('<Return>', app.find_path)
    root.bind('r', app.reset)

    root.mainloop()
