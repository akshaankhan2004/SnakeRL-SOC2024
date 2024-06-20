import numpy as np

class MazeSolver:
    def __init__(self, maze, discount_factor = 0.95, value_iters = 1000):
        self.maze = maze
        self.discount = discount_factor
        self.value_table = np.zeros((maze.shape[0], maze.shape[1]))
        self.reward_table = np.zeros((maze.shape[0], maze.shape[1]))
        self.initialise()
        for i in range(value_iters):
            self.value_table_update()
    
    def initialise(self):
        for i in range(self.maze.shape[0]):
            for j in range(self.maze.shape[1]):
                if self.maze[i][j] == 1:
                    self.value_table[i][j] = -100
                else:
                    continue
                
        for i in range(self.maze.shape[0]):
            for j in range(self.maze.shape[1]):
                if self.maze[i][j] == 1:
                    self.reward_table[i][j] = -100
                elif self.maze[i][j] == 3:
                    self.reward_table[i][j] = 1
                elif self.maze[i][j] == 2:
                    self.reward_table[i][j] = -1
                else:
                    continue
        
    def is_valid_state(self, state):
        if state[0] >= self.maze.shape[0] or state[0] < 0 or state[1] >= self.maze.shape[1] or state[1] < 0:
            return False
        if self.maze[state[0]][state[1]] == 1:
            return False
        else:
            return True
        
    def get_max_value(self, state):
        if not self.is_valid_state(state):
            return False
        else:
            values = []
            if self.is_valid_state((state[0] + 1, state[1])):
                v = self.value_table[state[0] + 1][state[1]] * self.discount + self.reward_table[state[0]][state[1]]
                values.append(v)
            if self.is_valid_state((state[0], state[1] + 1)):
                v = self.value_table[state[0]][state[1] + 1] * self.discount + self.reward_table[state[0]][state[1]]
                values.append(v)
            if self.is_valid_state((state[0] - 1, state[1])):
                v = self.value_table[state[0] - 1][state[1]] * self.discount + self.reward_table[state[0]][state[1]]
                values.append(v)
            if self.is_valid_state((state[0], state[1] - 1)):
                v = self.value_table[state[0]][state[1] - 1] * self.discount + self.reward_table[state[0]][state[1]]
                values.append(v)
            max_val = max(values)
            self.value_table[state[0]][state[1]] = max_val
        return True
    
    def get_best_state(self, state):
        x, y = state
        possible_states = [(x + 1, y), (x, y + 1), (x - 1, y), (x, y - 1)]
        values = []
        for s in possible_states:
            if not self.is_valid_state(s):
                values.append(-100)
            else:
                values.append(self.value_table[*s])
        index = np.array(values).argmax()
        return possible_states[index]

    def value_table_update(self):
        for i in range(self.maze.shape[0]):
            for j in range(self.maze.shape[1]):
                state = (i, j)
                update = self.get_max_value(state)
    
    def solve_maze(self, starting_state):
        value = 0
        curr_state = starting_state
        final_state = np.where(self.maze == 3)
        while not curr_state == final_state:
            if not self.is_valid_state(curr_state):
                print("INVALID STATE REACHED\nSTATE : ", curr_state)
                return False
            value += self.value_table[*curr_state]
            self.maze[*curr_state] = +100
            # value[*curr_state] = -1000
            curr_state = self.get_best_state(curr_state)
        return True
    
if __name__ == "__main__":
    grid_path = "../base/data/maze/grid10.txt"
    maze = np.loadtxt(grid_path)
    solver = MazeSolver(maze)
    solver.solve_maze((1, 1))
    print("MAZE SOLVED:\n")
    print(solver.maze)
    print("\n\nVALUE FUNCTION:\n")
    print(solver.value_table)
    