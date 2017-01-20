from copy import copy, deepcopy;
import math
import time

#Globals Used to Update and access data.
USE_DIAGONALS = True;
DIAGONAL_COST = math.sqrt(2);

char_map = [];
path_map = [];
state_map = [];
dest_location = (0, 0);

def goal_function(state):
    return state.map_rep == 'd';

def h_manhattan_distance(loc):
    return (abs(loc[0] - dest_location[0])) + (abs(loc[1] - dest_location[1]));

def h_euclidean_distance(loc):
    return math.sqrt(abs(loc[0] - dest_location[0])**2) + (abs(loc[1] - dest_location[1])**2);

#Used tp reset all the states in a map. Used when TEST_ALL_CASES is true. 
def refresh_state_map():
    for i in range(0, len(state_map)):
        for j in range(0, len(state_map[i])):
            state_map[i][j].reset_state();

class State:
    def __init__(self, map_rep, loc, map_size):
        self.map_rep = map_rep; #character representaion on the map file
        self.map_size = map_size; #tuple: (width, length)
        self.key = 0;
        self.path_cost = 0;
        self.time_expanded = 0; #used to generate the path.
        self.loc = loc; #tuple: (x, y)
        self.successors = [];
        self.accessed_diagonally = False; #was parent adjacent diagonally.
        self.JPS_pruned = False; #if pruned, then consider it as a wall.

    def __str__(self):
        "String representation."
        return "( id:" + str(self.get_hash_id()) + ", key: " + str(self.key) + " )";
 
    def __repr__(self):
        "String representation."
        return "( id:" + str(self.get_hash_id()) + ", key: " + str(self.key) + " )";

    #Resets the state
    def reset_state(self):
        self.successors = [];
        self.accessed_diagonally = False;
        self.key = 0;
        self.path_cost = 0;
        self.time_expanded = 0;
        self.JPS_pruned = False;
        
    '''    
    Calculates the f, or cost of the function where f = g + h
    path_cost: used to update state if a short path to state was found.
    '''
    def calculate_f(self, path_cost):
        g = 1;
        if self.accessed_diagonally:
            g = DIAGONAL_COST;
        if(self.map_rep == '@' or self.map_rep == 'T'):
            g = self.map_size[0] * self.map_size[1] * 1000; #simulate infinity for walls.
        if(self.map_rep == 'w'):
            g += 3;
        if(self.map_rep == 'm'):
            g += 5;

        g = g + path_cost;
        self.path_cost += g;
        h = heuristic_func(self.loc);
        self.key = g + h;

    '''
    Expands all states that are reachable and adjacent.
    '''
    def get_successors(self):
        if(len(self.successors) == 0):
            loc = self.loc;
            locations = [];
            locations.append((loc[0] - 1, loc[1], False));
            locations.append((loc[0], loc[1] - 1, False));
            locations.append((loc[0] + 1, loc[1], False));
            locations.append((loc[0], loc[1] + 1, False));
            #DIAGONALS
            if USE_DIAGONALS:
                locations.append((loc[0] - 1, loc[1] - 1, True));
                locations.append((loc[0] - 1, loc[1] + 1, True));
                locations.append((loc[0] + 1, loc[1] - 1, True));
                locations.append((loc[0] + 1, loc[1] + 1, True));

            #Check all state if they are a wall or reachable.
            for (x, y, diagonal) in locations:
                if(self._in_range(x, y, False)):
                    self.successors.append(state_map[x][y]);
                    state_map[x][y].accessed_diagonally = diagonal;

        return self.successors;

    '''
    Expands all states that are reachable and adjacent,
    then tries to "jump" from that expanded state.
    '''
    def get_successors_JPS(self):
        if(len(self.successors) == 0):
            loc = self.loc;
            locations = [];
            #DIAGONALS
            if USE_DIAGONALS:
                locations.append((loc[0] - 1, loc[1] - 1, True));
                locations.append((loc[0] - 1, loc[1] + 1, True));
                locations.append((loc[0] + 1, loc[1] - 1, True));
                locations.append((loc[0] + 1, loc[1] + 1, True));
            locations.append((loc[0] - 1, loc[1], False));
            locations.append((loc[0], loc[1] - 1, False));
            locations.append((loc[0] + 1, loc[1], False));
            locations.append((loc[0], loc[1] + 1, False));

            #Check all state if they are a wall or reachable.
            for (x, y, diagonal) in locations:
                if(self._in_range(x, y, True)):
                    #Try to jump.
                    path_cost = 1;
                    if(diagonal):
                        path_cost = DIAGONAL_COST;
                    jump_state = self._jump_successor(self, x - self.loc[0], y - self.loc[1], diagonal, path_cost);
                    if(jump_state != None):
                        jump_state.JPS_pruned = True;
                        self.successors.append(jump_state);
                        state_map[x][y].accessed_diagonally = diagonal;
        return self.successors;
    
    '''
    Jumps to the next adjacent node if no forced neighbours were found.
    diretion_state: our parent node.
    dx, dy: change in movement from direction_state to our expanded state.
            Ex: dx = 1, dy = 1. We are moving diagonally down and right.
    diagonal: whether we are moving diagonal. Used for pruning cases.
    curr_path_cost: keep track of the cost of moving when we jump.
                    JUMPS ARENT FREE!

    returns a state. None if we go out of range when expanding or if
                     found state is a dead end.    
    '''
    def _jump_successor(self, direction_state, dx, dy, diagonal, curr_path_cost):
        currX = direction_state.loc[0];
        currY = direction_state.loc[1];

        nextX = currX + dx;
        nextY = currY + dy;

        #Update path cost as we move through states
        new_path_cost = curr_path_cost;
        if(diagonal):
            new_path_cost += DIAGONAL_COST;
        else:
            new_path_cost += 1;
            
        #Is next movement a wall or out of bounds?
        if(not self._in_range(nextX, nextY, True)):
            if(diagonal):
                direction_state.path_cost = curr_path_cost;
                return direction_state;
            else:
                direction_state.JPS_pruned = True;
                return None;
        #If clear, check if it has been pruned before continuing.
        else:
            if(state_map[nextX][nextY].JPS_pruned):
                return None;

        #Is next movement the goal?
        if(goal_function(state_map[nextX][nextY])):
            state_map[nextX][nextY].path_cost = curr_path_cost;
            return state_map[nextX][nextY];

        #Check for forced neighbours
        forced_neighbour = False;
        if(diagonal):
            neighbour_1 = ((nextX - (dx)), nextY);
            neighbour_2 = (nextX, (nextY - (dy)));
            if(not self._in_range(neighbour_1[0], neighbour_1[1], True)):
                forced_neighbour = True;
            elif(not self._in_range(neighbour_2[0], neighbour_2[1], True)):
                forced_neighbour = True;
        else:
            forced_neighbour = self._has_forced_neighbours(currX, currY, dx, dy, nextX, nextY);

        #Diagonal Case for forced neighbours
        if(diagonal):
            if(forced_neighbour):
                return state_map[nextX][nextY];
            horizontal_node = self._jump_successor(state_map[nextX][nextY], dx, 0, False, new_path_cost);
            vertical_node = self._jump_successor(state_map[nextX][nextY], 0, dy, False, new_path_cost);
            if(horizontal_node != None or vertical_node != None):
                state_map[nextX][nextY].path_cost = curr_path_cost;
                return state_map[nextX][nextY];
            else:
                direction_state.JPS_pruned = True;
                
        #Horizantal and Vertical Case for forced neighbours
        else:
            if(forced_neighbour):
                state_map[nextX][nextY].path_cost = curr_path_cost;
                return state_map[nextX][nextY];

        #If nothing blocking or no forced neighbours, continue down path.     
        return self._jump_successor(state_map[nextX][nextY], dx, dy, diagonal, new_path_cost);
    
    '''
    Looks at our neighbours to see if they are all optimally
    reachable via parent node 'p' or next node 'x'.
    
    Ex 1:
    [1][3][5]
    [p][x][ ]
    [2][4][6]
    1)In this case 1 and 2 are trivially optimally reachable from p
    because they are adjacent. Therefore we dont check them.
    2)3 and 4 can be reached optimally by going diagonal from p.
    3)5 and 6 can be reached optimally by going from p->x->5 or 6.

    Ex 2:
    [1][@][5]
    [p][x][ ]
    [2][4][@]
    1)In this case 1 and 2 are trivially optimally reachable ...
    2)5 cannot be reached optimally by going diagonal from p.
    this is called a forced neighbour. Although we can get to 5
    from x, we cannot guarentee optimality for 5.
    3)4 has a similar problem but we personnally chose to consider
    this as a future case and therefore a forced neighbour.

    Ex 3:
    [1][@][@]
    [p][x][ ]
    [2][@][@]
    1)In this case 1 and 2 are trivially optimally reachable...
    2)Considered to be the same case as Ex 1, except there are
    no neighbours so we dont need to concern ourselves with them.

    returns True if forced neighbour is found. False otherwise.
    '''
    def _has_forced_neighbours(self, cX, cY, dx, dy, nX, nY):
        
        curr_neighbour_1_x = (cX + (dy));
        curr_neighbour_1_y = (cY + (dx));
                              
        curr_neighbour_2_x = (cX - (dy));
        curr_neighbour_2_y = (cY - (dx));

        next_neighbour_1_x = (nX + (dy));
        next_neighbour_1_y = (nY + (dx));
                              
        next_neighbour_2_x = (nX - (dy));
        next_neighbour_2_y = (nY - (dx));

        c_1 = self._in_range(curr_neighbour_1_x, curr_neighbour_1_y, True);
        n_1 = self._in_range(next_neighbour_1_x, next_neighbour_1_y, True);
        if(c_1 != n_1):
            return True;
        
        c_2 = self._in_range(curr_neighbour_2_x, curr_neighbour_2_y, True);
        n_2 = self._in_range(next_neighbour_2_x, next_neighbour_2_y, True);
        if(c_2 != n_2):
            return True;
            
        return False;

    '''
    Checks to see if a location on map is reachable (not map[-1][0])
    x, y: location on the map.
    use_wall_check: If true, considers any '@' or 'T' as a
                    non-reachable state.
    returns True if the node is expandable. False otherwise.
    '''
    def _in_range(self, x, y, use_wall_check):
        index_check = (x >= 0 and x <= self.map_size[0] - 1) and (y >= 0 and y <= self.map_size[1] - 1);
        wall_check = True;
        if(index_check):
            wall_check = state_map[x][y].map_rep != 'T' and state_map[x][y].map_rep != '@';
        if(not use_wall_check):
            wall_check = True;
        return  index_check and wall_check;
    
    '''
    Checks to see if a location on map is reachable (not map[-1][0])
    then checks if it is a wall.
    x, y: location on the map.
    returns True if the node is a Wall. False otherwise.
    '''
    def _is_wall(self, x, y):
        index_check = (x >= 0 and x <= self.map_size[0] - 1) and (y >= 0 and y <= self.map_size[1] - 1);
        wall_check = False;
        if(index_check):
            wall_check = state_map[x][y].map_rep == 'T' or state_map[x][y].map_rep == '@';
        else:
            wall_check = False;
        return wall_check;

    '''
    returns the unique id for the state. (Ex '@50_45')
    '''
    def get_hash_id(self):
        return self.map_rep + str(self.loc[0]) + "_" + str(self.loc[1]);  
    
class SearchEngine:
    def __init__(self, goal_fcn):
        self.goal_fcn = goal_fcn;

    '''
    Searches using A*.
    We use a Closed dictionary for our cycle checking.
    '''
    def search(self, initState):
        Closed = dict();
        Open = [];
        Open.append(initState);
        expanded = 0;
        curr_expanded_path = [];
        while len(Open) != 0:
            min_key = Open[0].key;
            curr_index = 0;
            curr_state = Open[0];
            
            #extract state with min f-value / key
            for i in range(0, len(Open)):
                if Open[i].key < min_key:
                    min_key = Open[i].key;
                    curr_index = i;
                    curr_state = Open[i];
            del Open[curr_index];

            #Add node to path
            if(curr_state.time_expanded > len(curr_expanded_path) - 1):
                curr_expanded_path.append(curr_state);
            else:
                curr_expanded_path[curr_state.time_expanded] = curr_state;
            
            if(self.goal_fcn(curr_state)):
                print("solution found with: " + str(expanded) + " nodes expanded.");
                print("with path length: " + str(curr_state.time_expanded));
                return curr_state;
            
            else:
                #mark expansion on map.
                path_map[curr_state.loc[0]][curr_state.loc[1]] = "x";

                #add to closed dict so that we dont expand this node again.
                Closed[curr_state.get_hash_id()] = curr_state.path_cost;
                
                expanded += 1;
                if JPS:
                    states = curr_state.get_successors_JPS();
                    curr_state.JPS_pruned = True;
                else:
                    states = curr_state.get_successors();
                    
                #expand curr and add to Open
                for s in states:
                    s.time_expanded = curr_state.time_expanded + 1;
                    if not (s.get_hash_id() in Closed.keys()):
                        s.calculate_f(curr_state.path_cost);
                        Open.append(s);
                        Closed[s.get_hash_id()] = s.path_cost;
                    else:
                        
                        #Update nodes cost if cheaper.
                        path_cost = 1;
                        if(s.accessed_diagonally):
                            path_cost = DIAGONAL_COST;
                        if(Closed[s.get_hash_id()] > curr_state.path_cost + path_cost):
                            
                            #recalculate costs
                            s.path_cost = 0;
                            s.calculate_f(curr_state.path_cost);
                            Closed[s.get_hash_id()] = s.path_cost;
                            
        print("no solution found");


#TEST A* SEARCH AND JPS----------------------------------------------

#TESTABLE MAPS============
#Small size map tests
MAP_SMALL_EASY = "map_1.map";
MAP_SMALL_MEDIUM = "map_3.map";
MAP_SMALL_HARD = "map_2.map";

#Large size map tests
MAP_LARGE_MEDIUM = "map_large_medium.map";
MAP_LARGE_HARD_1 = "map_large_hard.map";
MAP_LARGE_HARD_2 = "map_large_hard_2.map";
MAP_LARGE_HARD_3 = "map_large_hard_3.map";

#========================

MAP_PATH_OUTPUT = "map_path_output.txt";

TEST_ALL_CASES = True;

#heuristic functions: h_manhattan_distance , h_euclidean_distance.
heuristic_func = h_manhattan_distance;
JPS = False;

#Set the map file you want to test:
TEST_FILE = MAP_LARGE_HARD_1;


#Read file and setup char_map
with open(TEST_FILE) as map_file:
    for line in map_file:
        char_nodes = [];
        
        for char in line:
            if(char != '\n'):
                char_nodes.append(char);
                
        char_map.append(char_nodes);
        #print(char_map[len(char_map) - 1]);
map_file.close;

#Setup state_map
init_state = None;
for i in range(0, len(char_map)):
    state_nodes = [];
    for j in range(0, len(char_map[i])):
        s = State(char_map[i][j], (i,j), (len(char_map), len(char_map[i])));
        if(char_map[i][j] == 's'):
            init_state = s;
        if(char_map[i][j] == 'd'):
            dest_location = (i , j);
        state_nodes.append(s);
    state_map.append(state_nodes);
        
#Search
if(TEST_ALL_CASES):
    path_map = deepcopy(char_map);
    
    print("Manhattan, no JPS: \n");
    heuristic_func = h_manhattan_distance;

    
    se = SearchEngine(goal_function);
    start_time = time.clock();
    se.search(init_state);
    end_time = time.clock();
    print("Completed in: " + str(end_time - start_time) + " seconds." + "\n");
    refresh_state_map();

    print("Euclidean, no JPS: \n");
    heuristic_func = h_euclidean_distance;
    
    se = SearchEngine(goal_function);
    start_time = time.clock();
    se.search(init_state);
    end_time = time.clock();
    print("Completed in: " + str(end_time - start_time) + " seconds." + "\n");
    refresh_state_map();

    #Start testing with JPS
    JPS = True;
    
    print("Manhattan, with JPS: \n");
    heuristic_func = h_manhattan_distance;
    
    se = SearchEngine(goal_function);
    start_time = time.clock();
    se.search(init_state);
    end_time = time.clock();
    print("Completed in: " + str(end_time - start_time) + " seconds." + "\n");
    refresh_state_map();

    print("Euclidean, with JPS: \n");
    heuristic_func = h_euclidean_distance;
    
    se = SearchEngine(goal_function);
    start_time = time.clock();
    se.search(init_state);
    end_time = time.clock();
    print("Completed in: " + str(end_time - start_time) + " seconds." + "\n");
    refresh_state_map();
else:  
    path_map = deepcopy(char_map);
    se = SearchEngine(goal_function);
    start_time = time.clock();
    se.search(init_state);
    end_time = time.clock();
    print("Completed in: " + str(end_time - start_time) + " seconds.");

#Reset output file.
with open(MAP_PATH_OUTPUT, "w"):
        pass

#Write path to output file.
with open(MAP_PATH_OUTPUT, "w") as f_o:
    for i in path_map:
        for j in i:
            f_o.write(j);
        f_o.write("\n");

    
