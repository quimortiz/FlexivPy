import numpy as np


class RRT:
    def __init__(
        self,
        goal_bias=0.2,
        collision_resolution=0.1,
        max_step=0.5,
        max_it=200,
        is_collision_free=lambda x: True,
        sample_fun=lambda: np.random.rand(7) * 2 * np.pi - np.pi,
        goal_tolerance=1e-3,
    ):

        self.collision_resolution = collision_resolution
        self.goal_bias = goal_bias
        self.max_step = max_step
        self.tree = []
        self.parents = []
        self.max_it = max_it
        self.is_collision_free = is_collision_free
        self.sample_fun = sample_fun
        self.goal_tolerance = goal_tolerance

    def get_nearest_neighbor_id(self, q):
        min_distance = np.inf
        min_index = -1
        for i, qtree in enumerate(self.tree):
            distance = np.linalg.norm(q - qtree)
            if distance < min_distance:
                min_distance = distance
                min_index = i
        return min_index

    def is_collision_free_edge(self, q1, q2):
        num_checks = int(np.linalg.norm(q1 - q2) / self.collision_resolution)
        for i in range(num_checks):
            qcheck = q1 + i * (q2 - q1) / num_checks
            if not self.is_collision_free(qcheck):
                return False
        return True

    def solve(self, start, goal):
        self.home = start
        self.goal = goal
        solved = False
        goal_id = -1

        self.tree.append(self.home)
        self.parents.append(-1)

        if not self.is_collision_free(self.home):
            raise Exception("Initial configuration is in collision")

        if not self.is_collision_free(self.goal):
            raise Exception("Goal configuration is in collision")

        for _ in range(self.max_it):

            # sample a random configuration

            if np.random.rand() < self.goal_bias:
                qrand = self.goal
            else:
                valid_sample = False
                while not valid_sample:
                    qrand = self.sample_fun()
                    valid_sample = self.is_collision_free(qrand)

                if not valid_sample:
                    raise Exception("Could not find a valid sample")

            # Get nearest neighbor
            nn_id = self.get_nearest_neighbor_id(qrand)
            qnear = self.tree[nn_id]

            # do a step
            if np.linalg.norm(qrand - qnear) > self.max_step:
                qnew = qnear + self.max_step * (qrand - qnear) / np.linalg.norm(
                    qrand - qnear
                )
            else:
                qnew = qrand

            # check if the edge is collisoin free

            if self.is_collision_free_edge(qnear, qnew):
                self.tree.append(qnew)
                self.parents.append(nn_id)

                if np.linalg.norm(qnew - self.goal) < self.goal_tolerance:
                    goal_id = len(self.tree) - 1
                    solved = True
                    break

        if solved:
            # trace back the solution
            id = goal_id
            path = []
            while id != -1:
                path.append(id)
                id = self.parents[id]

            # revese the path
            path = path[::-1]
            # return the corresponding configurations.
            return [self.tree[id] for id in path]

        else:
            raise Exception("Could not find a solution")
