import sys
from collections import deque, defaultdict
import heapq

CellPosition = tuple[int, int]


def read_maze():
    return [list(line.strip("\n")) for line in sys.stdin]


def find_objects(grid):
    robot_starts = []
    key_locations = {}
    for row_idx, row in enumerate(grid):
        for col_idx, cell in enumerate(row):
            if cell == '@':
                robot_starts.append((row_idx, col_idx))
            elif 'a' <= cell <= 'z':
                key_locations[cell] = (row_idx, col_idx)
    return robot_starts, key_locations


def prepare_nodes(robot_starts, key_locations):
    sorted_keys = sorted(key_locations)
    important_nodes = [*robot_starts] + [key_locations[k] for k in sorted_keys]
    node_to_id = {pos: idx for idx, pos in enumerate(important_nodes)}
    return important_nodes, node_to_id


def create_path_graph(grid, nodes, node_mapping):
    graph = [defaultdict(list) for _ in range(len(nodes))]
    rows, cols = len(grid), len(grid[0])

    for source_id, (start_row, start_col) in enumerate(nodes):
        visited = {(start_row, start_col): [0]}
        queue = deque([(start_row, start_col, 0, 0)])

        while queue:
            row, col, steps, door_state = queue.popleft()
            target_id = node_mapping.get((row, col))

            if target_id not in (None, source_id):
                graph[source_id][target_id].append((door_state, steps))

            for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                new_row, new_col = row + dr, col + dc
                if not (0 <= new_row < rows and 0 <= new_col < cols):
                    continue
                if grid[new_row][new_col] == '#':
                    continue

                new_door_state = door_state
                cell = grid[new_row][new_col]
                if 'A' <= cell <= 'Z':
                    new_door_state |= 1 << (ord(cell.lower()) - ord('a'))

                existing_states = visited.get((new_row, new_col), [])
                if any((state & new_door_state) == state for state in existing_states):
                    continue

                valid_states = [s for s in existing_states
                                if not (new_door_state & s == new_door_state)] + [new_door_state]
                visited[(new_row, new_col)] = valid_states
                queue.append((new_row, new_col, steps + 1, new_door_state))
    return graph


def optimize_graph(graph):
    for source in range(len(graph)):
        for target in list(graph[source].keys()):
            best_masks = {}
            for mask, dist in graph[source][target]:
                if mask not in best_masks or dist < best_masks[mask]:
                    best_masks[mask] = dist

            final = []
            for mask in best_masks:
                if not any(other != mask and (other & mask) == other
                           and best_masks[other] <= best_masks[mask] for other in best_masks):
                    final.append((mask, best_masks[mask]))
            graph[source][target] = final


def get_min_edge(graph):
    return min((d for connections in graph for paths in connections.values()
                for _, d in paths), default=0)


def find_optimal_path(graph, total_keys):
    target_mask = (1 << total_keys) - 1
    initial_state = (0, 1, 2, 3, 0)
    min_step = get_min_edge(graph)
    heap = []
    heapq.heappush(heap, (0, 0, initial_state))
    best_states = {initial_state: 0}

    while heap:
        priority, cost, state = heapq.heappop(heap)
        if priority != 0:  # Priority queue optimization
            continue
        if state[4] == target_mask:
            return cost

        robots, collected = state[:4], state[4]
        for robot_idx, current_pos in enumerate(robots):
            for key_node in range(4, len(graph)):
                key_flag = 1 << (key_node - 4)
                if collected & key_flag:
                    continue

                for required_doors, distance in graph[current_pos].get(key_node, []):
                    if required_doors & ~collected:
                        continue

                    new_collected = collected | key_flag
                    updated_robots = list(robots)
                    updated_robots[robot_idx] = key_node
                    new_state = (*updated_robots, new_collected)
                    new_cost = cost + distance
                    if new_cost < best_states.get(new_state, 999999):
                        best_states[new_state] = new_cost
                        remaining = total_keys - bin(new_collected).count('1')
                        estimate = new_cost + remaining * min_step
                        heapq.heappush(heap, (estimate, new_cost, new_state))
    return -1


def solve_maze(grid):
    starts, keys = find_objects(grid)
    nodes, node_map = prepare_nodes(starts, keys)
    graph = create_path_graph(grid, nodes, node_map)
    optimize_graph(graph)
    return find_optimal_path(graph, len(keys))


def main():
    print(solve_maze(read_maze()))


if __name__ == '__main__':
    main()
