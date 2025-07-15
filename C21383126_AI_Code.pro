% Maze 1
% Wall squares
%size(5,5). % 5x5 grid
%wall((2,2)).
%wall((2,3)).
%wall((2,4)).
%wall((3,4)).
%wall((4,3)).
%wall((4,4)).

% Start and Goal
%start((3,3)).
%goal((1,5)).

% Water Squares
%water((1,3)).
%water((4,2)).

% Maze 2

size(10,10).

% Wall squares
wall((2,4)).
wall((3,4)).
wall((4,4)).
wall((5,4)).
wall((6,4)).
wall((7,4)).
wall((8,4)).
wall((9,4)).
wall((10,4)).
wall((6,5)).
wall((6,6)).
wall((6,7)).
wall((6,8)).
wall((6,9)).
wall((7,7)).
wall((8,7)).
wall((9,7)).

% Start and Goal
start((1,1)).
goal((8,6)).

% Water Squares
water((1,7)).
water((2,5)).
water((5,10)).
water((6,2)).
water((10,8)).


% Valid moves - replaces edge/2 in search algorithms
valid((X,Y)) :-
    size(W,H),
    X >= 1, X =< W,
    Y >= 1, Y =< H,
    not(wall((X,Y))).

move((X,Y), (X1,Y)) :-
    % Move Right
    X1 is X+1,
    valid((X1,Y)).

move((X,Y), (X1,Y)) :-
    % Move Left
    X1 is X-1,
    valid((X1,Y)).

move((X,Y), (X,Y1)) :-
    % Move Up
    Y1 is Y+1,
    valid((X,Y1)).

move((X,Y), (X,Y1)) :-
    % Move Down
    Y1 is Y-1,
    valid((X,Y1)).

% Cost of each move
cost((X,Y), 1) :-
    valid((X,Y)),
    not(water((X,Y))).

cost((X,Y), 3) :-
    valid((X,Y)),
    water((X,Y)).

% -------------------------------------------------------------------
% Depth First Search
dfs(Node, Solution) :-
    depthfirst([], Node, Solution).

% Base case: reached the goal
depthfirst(Path, Node, [Node | Path]) :-
    goal(Node).

depthfirst(Path, Node, Sol) :-
    move(Node, Node1), % Check if it's a valid move
    not(member(Node1, Path)), % avoids loops
    depthfirst([Node | Path], Node1, Sol).

% -------------------------------------------------------------------
% Breadth First Search

extend([Node | Rest], Visited, NewPaths, NewVisited) :-
    findall([Next, Node | Rest],
        (move(Node, Next), not(member(Next, Visited))),
        NewPaths),
    extract_heads(NewPaths, Heads),
    append(Visited, Heads, NewVisited).

% Goal found
breadthfirst([[Goal | Rest] | _], Goal, _, [Goal | Rest]).

breadthfirst([Path | OtherPaths], Goal, Visited, Result) :-
    extend(Path, Visited, NewPaths, NewVisited),
    append(OtherPaths, NewPaths, UpdatedPaths),
    breadthfirst(UpdatedPaths, Goal, NewVisited, Result).

% Entry point
bfs(Start, Goal, Path) :-
    breadthfirst([[Start]], Goal, [Start], RevPath),
    reverse(RevPath, Path).

% Get visited nodes from new paths
extract_heads([], []).
extract_heads([[H|_] | T], [H | Rest]) :-
    extract_heads(T, Rest).

% -------------------------------------------------------------------
% Iterative-Deepening Search

ids(Start, Goal, Path) :-
    between(0, inf, DepthLimit),
    dls(Start, Goal, [Start], Path, DepthLimit).

% If at goal, return path
dls(Goal, Goal, Path, Path, _).

% Explore neighbours if not at depth limit
dls(Node, Goal, PathSoFar, FinalPath, MaxDepth) :-
    MaxDepth > 0,
    move(Node, Next),
    not(member(Next, PathSoFar)), % Avoid loops
    NewDepth is MaxDepth - 1,
    dls(Next, Goal, [Next | PathSoFar], FinalPath, NewDepth).

% --------------------------------------------------------------------
% Get energy cost of path
path_cost([_], 0). % If the path is just the start
% Only add cost of path without the start square
path_cost([_|Rest], Cost) :-
    path_cost_add(Rest, Cost).

% Add cost of the rest of the squares
path_cost_add([], 0).
path_cost_add([Pos | Rest], Cost) :-
    cost(Pos, StepCost),
    path_cost_add(Rest, RestCost),
    Cost is StepCost + RestCost.

% ---------------------------------------------------------------------
% A* Search
% Heuristic: Manhattan distance between any (X,Y) and the goal
h((X,Y), H) :-
    goal((GX,GY)),
    H is abs(X-GX) + abs(Y-GY).

% Move one step
move_astar([Node | Path] / G /_, [NextNode, Node | Path] / NewG / HNext) :-
    move(Node, NextNode),
    not(member(NextNode, Path)),
    cost(NextNode, StepCost),
    NewG is G + StepCost,
    h(NextNode, HNext).

% Expand one path into all possible next steps
expand_astar(Path, ExpandedPaths) :-
    findall(NewPath, move_astar(Path, NewPath), ExpandedPaths).

% Select the best path with the smallest f = g + h
get_best([Path], Path) :- !.
get_best([P1 / G1 / H1, _ / G2 / H2 | Ps], Best) :-
    G1 + H1 =< G2 + H2, !,
    get_best([P1 / G1 / H1 | Ps], Best).
get_best([_ | Ps], Best) :-
    get_best(Ps, Best).

% Base case: the best path reaches the goal
astar(Paths, _, Path) :-
    get_best(Paths, Path),
    Path = [Node | _] / _ / _,
    goal(Node).

% Skip already visited end nodes to avoid failure
astar(Paths, Visited, Solution) :-
    get_best(Paths, Best),
    Best = [Node | _] / _ / _,
    member(Node, Visited),
    select(Best, Paths, Rest),
    astar(Rest, Visited, Solution).

% Recursive case: expand the best unvisited path
astar(Paths, Visited, Solution) :-
    get_best(Paths, Best),
    Best = [Node | _] / _ / _,
    not(member(Node, Visited)), % only expand if not yet visited
    select(Best, Paths, OtherPaths),
    expand_astar(Best, NewPaths),
    append(NewPaths, OtherPaths, AllPaths),
    astar(AllPaths, [Node | Visited], Solution).

% Entry point
solve_astar(Start, FinalPath / Cost) :-
    h(Start, H),
    astar([[Start] / 0 / H], [], RevPath / Cost / _),
    reverse(RevPath, FinalPath).





