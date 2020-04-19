import sys
import math

# Simple euclid distance calculation
def distance(a, b):
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    return (dx ** 2 + dy ** 2) ** 0.5

def get_aim(a, b, bv=(0,0)):
    return (b[0] + bv[0] - a[0], b[1] + bv[1] - a[1])

def add_vectors(a, b):
    return (a[0] + b[0], a[1] + b[1])

def subtract_vectors(a, b):
    return (a[0] - b[0], a[1] - b[1])

def create_trajectory(point, vec, rounds, friction):
	end_x = point[0]
	end_y = point[1]
	dx = vec[0]
	dy = vec[1]
	for i in range(rounds):
		end_x += dx
		end_y += dy
		dx *= friction
		dy *= friction
	return (point, (int(end_x), int(end_y)))
	
def normalize_vector(vec):
    l = (vec[0] ** 2 + vec[1] ** 2) ** 0.5
    if l == 0:
        return (0, 0)
    norm = (vec[0] / l, vec[1] / l)
    return norm

def multiply_vector(vec, c):
    return (round(c * vec[0]), round(c * vec[1]))

def vector_magnitude(vec):
    return (vec[0] ** 2 + vec[1] ** 2) ** 0.5

def vector_dot(v1, v2):
    return v1[0] * v2[0] + v1[1] * v2[1]

def vector_angle(v):
    a = math.atan2(v[1], v[0])
    return a

def collision_angle(v1, v2):
    a = vector_angle(normalize_vector(v1))
    b = vector_angle(normalize_vector(v2))
    return normalize_angle(b - a)

def normalize_angle(angle):
    a = angle
    while a < -math.pi:
        a += 2*math.pi
    while a > math.pi:
        a -= 2*math.pi
    return a

def right_angle_vectors(vec):
    cw = (vec[1], -1 * vec[0])
    ccw = (-1 * vec[1], vec[0])
    return cw, ccw

def vector_rotate(vec, angle):
    x = vec[0] * math.cos(angle) - vec[1] * math.sin(angle)
    y = vec[0] * math.sin(angle) + vec[1] * math.cos(angle)
    return (round(x), round(y))

def segment_intersection(line1, line2):
	def on_segment(a, b, c):
		return min(a[0], c[0]) <= b[0] <= max(a[0], c[0]) and min(a[1], c[1]) <= b[1] <= max(a[1], c[1])

	def orient(a, b, c):
		val = (float(b[1] - a[1]) * (c[0]  - b[0])) - (float(b[0] - a[0]) * (c[1] - b[1]))
		if val > 0:
			return 1
		elif val < 0:
			return 2
		else:
			return 0
	
	o1 = orient(line1[0], line1[1], line2[0])
	o2 = orient(line1[0], line1[1], line2[1])
	o3 = orient(line2[0], line2[1], line1[0])
	o4 = orient(line2[0], line2[1], line1[1])

	if o1 != o2 and o3 != o4:
		return True
	elif o1 == 0 and on_segment(line1[0], line2[0], line1[1]):
		return True
	elif o2 == 0 and on_segment(line1[0], line2[1], line1[1]):
		return True
	elif o3 == 0 and on_segment(line2[0], line1[0], line2[1]):
		return True
	elif o4 == 0 and on_segment(line2[0], line1[1], line2[1]):
		return True
	else:
		return False

def min_dist(line1, line2):
    def point_to_line(line, point):
        dx = line[1][0] - line[0][0]
        dy = line[1][1] - line[0][1]
        
        if dx == 0 and dy == 0:
            return distance(line[0], point)
            
        t = ((point[0] - line[0][0]) * dx + (point[1] - line[0][1]) * dy) / (dx ** 2 + dy ** 2)
        
        if t < 0:
            closest = line[0]
        elif t > 1:
            closest = line[1]
        else:
            closest = (line[0][0] + t * dx, line[0][1] + t * dy)
        return distance(point, closest)
        
    h1 = point_to_line(line1, line2[0])
    h2 = point_to_line(line1, line2[1])
    h3 = point_to_line(line2, line1[0])
    h4 = point_to_line(line2, line1[1])
    return min(h1, h2, h3, h4)

def simulate_move(wizzard, snaffle, thrust, round_limit):
    def future_vector(loc, vec, dest, thrust, mass, friction):
        move_vec = (dest[0] - loc[0], dest[1] - loc[1])
        norm = normalize_vector(move_vec)
        adj = (norm[0] * thrust / mass, norm[1] * thrust / mass)
        tot = (round((vec[0] + adj[0]) * friction), round((vec[1] + adj[1]) * friction))
        return tot
        
    snaff_vec = snaffle.new_vector((0, 0), 0)
    wizz_aim = get_aim(wizzard.vec, snaffle.loc, snaff_vec)
    wizz_vec = wizzard.new_vector(wizz_aim, thrust)
    
    wizz_tr = create_trajectory(wizzard.loc, wizz_vec, 1, wizzard.friction)
    snaff_tr = create_trajectory(snaffle.loc, snaff_vec, 1, snaffle.friction)
    md = min_dist(wizz_tr, snaff_tr)
    rnd = 1
    radius_limit = wizzard.radius
    while md >= radius_limit and rnd < round_limit:
        wizz_loc = wizz_tr[1]
        snaff_loc = snaff_tr[1]
        snaff_vec = future_vector(snaff_loc, snaff_vec, (0, 0), 0, snaffle.mass, snaffle.friction)
        wizz_aim = get_aim(wizz_vec, snaff_loc, snaff_vec)
        wizz_vec = future_vector(wizz_loc, wizz_vec, wizz_aim, thrust, wizzard.mass, wizzard.friction)
        wizz_tr = create_trajectory(wizz_loc, wizz_vec, 1, wizzard.friction)
        snaff_tr = create_trajectory(snaff_loc, snaff_vec, 1, snaffle.friction)
        md = min_dist(wizz_tr, snaff_tr)
        rnd += 1
    return rnd, wizz_vec 
    
def pass_aim(org, dest):
    # d = distance(org.loc, dest.loc)
    # r = 3
    # if d < 2600:
    #     r = (d / 1000) * 3
    # v = multiply_vector(dest.vec, r)
    aim = get_aim(org.vec, dest.loc, dest.aim_vec)
    return (round(aim[0]), round(aim[1]))

def check_inbonds(loc):
    if loc[0] < 0 or loc[0] > 16000:
        return False
    elif loc[1] < 0 or loc[1] > 7500:
        return False
    else:
        return True

class GameUnit():
    def __init__(self, uid, x, y, vx, vy, state):
        self.uid = uid
        self.loc = (x, y)
        self.vec = (vx, vy)
        self.state = 0
        self.mass = 1
        self.friction = 0.75
        self.radius = 400
        self.sector = 0
        self.thrust = 0
    
    def __str__(self):
        s = f"UID: {self.uid}\n"
        s += f"LOC: {self.loc}\n"
        s += f"VEC: {self.vec}\n"
        s += f"STA: {self.state}"
        return s
    
    def get_sector(self):
        if self.loc[0] < 5000:
            self.sector = 0
        elif self.loc[0] < 11000:
            self.sector = 1
        else:
            self.sector = 2

    def update(self, x, y, vx, vy, state):
        self.loc = (x, y)
        self.vec = (vx, vy)
        self.state = state
        self.get_sector()
    
    def new_vector(self, destination, thrust=-1):
        if thrust == -1:
            thrust = self.thrust
        move_vec = (destination[0] - self.loc[0], destination[1] - self.loc[1])
        norm = normalize_vector(move_vec)
        adj = (norm[0] * thrust / self.mass, norm[1] * thrust / self.mass)
        tot = (round((self.vec[0] + adj[0]) * self.friction), round((self.vec[1] + adj[1]) * self.friction))
        return tot

    def create_trajectory_const(self, rounds, vec=None, thrust=-1):
        end = self.loc
        if vec:
            nv = vec
        else:
            nv = self.new_vector(self.vec, thrust)
        for i in range(rounds):
            end = add_vectors(end, nv)
            nv = self.new_vector(nv, thrust)
        return (self.loc, (round(end[0]), round(end[1])))

    def check_for_collision(self, unit_dict, vec=None,rounds_limit=5, thrust=-1):
        for key, val in unit_dict.items():
            for rounds in range(rounds_limit):
                self_line = self.create_trajectory_const(rounds, vec, thrust)
                obj_line = val.create_trajectory_const(rounds, vec, thrust)
                if not check_inbonds(self_line[1]) or not check_inbonds(obj_line[1]):
                    break
                # Avoiding only 1 object atm
                md = min_dist(self_line, obj_line)
                # if md < 900:
                #     print(f"{self.uid} <-> {val.uid} at {rounds}: {md:.2f}", file=sys.stderr)
                if segment_intersection(self_line, obj_line) or md < (self.radius + val.radius):
                    s = subtract_vectors(self_line[1], self_line[0])
                    o = subtract_vectors(obj_line[1], obj_line[0])
                    cv = subtract_vectors(obj_line[1], self_line[1])
                    a = vector_angle(s)
                    cv = vector_rotate(cv, -1 * a)
                    return True, val.uid, rounds, collision_angle(s, o), vector_angle(cv)
        return False, None, None, None, None

class Wizzard(GameUnit):
    def __init__(self, uid, x, y, vx, vy, state):
        super().__init__(uid, x, y, vx, vy, state)
        self.aim_id = None
        self.aim_vec = None
        self.rounds_to_reach = None
        self.magic_id = None
        self.sector_pref = [0, 1, 2]
        self.role = None
        self.talk = f"S{self.aim_id}"
        self.thrust = 150

    def move(self, dest, thrust):
        print(f"MOVE {dest[0]} {dest[1]} {thrust}")
    
    def default_speech(self):
        self.talk = f"S{self.aim_id}"
    
    def move_to_object(self, dest, dest_vec, blud_dict, other_wiz):
        def test_move(blud_dict, vector, max_detect=9):
            col, uid, r, ang, cv = self.check_for_collision(blud_dict, vec, max_detect)
            if col and r > 1 and abs(ang) > math.pi / 2:
                 False
            True

        def adjust_move(aim, blud_dict, max_detect, limit=True):
            step = 1 if limit else 5
            bound = 90 if limit else 360
            for i in range(0, bound // 2, step):
                vector = vector_rotate(aim, math.radians(i))
                if test_move(blud_dict, vector, max_detect):
                    return True, vector
                vector = vector_rotate(aim, math.radians(-i))
                if test_move(blud_dict, vector, max_detect):
                    return True, vector
            return False, None
            
        print(f"Moving W{self.uid} to {dest}", file=sys.stderr)
        thrust = 150
        max_detect = 9
        aim = get_aim(self.vec, dest, dest_vec)
        vec = self.new_vector(aim, 150)
        # col, uid, r, ang, cv = self.check_for_collision({other_wiz.uid: other_wiz}, vec, max_detect)
        # if col:
        #     print(f"CW! W{self.uid} & W{uid} in {r}, {ang:.2f}", file = sys.stderr)
        #     self.talk += f"|CW! W{uid}({r},{ang:.2f})"
        col, uid, r, ang, cv = self.check_for_collision(blud_dict, vec, max_detect)
        if col and r > 1 and abs(ang) > math.pi / 2:
            ang_d = math.degrees(ang)
            print(f"CW! W{self.uid}({vec}) & BL {uid} in {r}, {ang:.2f}({ang_d:.2f}, {cv:.2f})", file = sys.stderr)
            t, target = adjust_move(aim, blud_dict, max_detect)
            if t:
                
                aim = target
        self.move(aim, thrust)
    
    def get_estimates(self, unit, rnd_limit=50):
        dist = distance(self.loc, unit.loc)
        rounds, final_vector = simulate_move(self, unit, self.thrust, rnd_limit)
        return dist, rounds, final_vector
    
    def uid_holding(self, snaffs):
        for key, val in snaffs.items():
            if val.loc == self.loc:
                return key
        return None

class Snaffle(GameUnit):
    def __init__(self, uid, x, y, vx, vy, state):
        super().__init__(uid, x, y, vx, vy, state)
        self.mass = 0.5
        self.radius = 0

class Bludger(GameUnit):
    def __init__(self, uid, x, y, vx, vy, state):
        super().__init__(uid, x, y, vx, vy, state)
        self.mass = 8
        self.friction = 1.9
        self.radius = 200
        self.thrust = 1000

# GoalPost 300 radius
# Left Goal: (X=0, Y=3750) -+2000 
# Right Goal: (X=16000, Y=3750) +-2000
class Match():
    def __init__(self, team_id):
        self.team_id = team_id
        self.rounds = 200
        if team_id:
            self.my_goal = 16000
            self.my_goal_loc = (16000, 3750)
            self.en_goal_loc = (0, 3750)
            self.enemy_goal = 0
            self.enemy_goal_spots = [(0, 3750), (0, 5380), (0, 2120)]
        else:
            self.my_goal = 0
            self.my_goal_loc = (0, 3750)
            self.en_goal_loc = (16000, 3750)
            self.enemy_goal = 16000
            self.enemy_goal_spots = [(16000, 3750), (16000, 5380), (16000, 2120)]
        self.goal_y = (2120, 5380)
        self.my_wizz1 = None
        self.my_wizz2 = None
        self.opp_wizz1 = None
        self.opp_wizz2 = None
        self.opp_wizzs = dict()
        self.bludgers = dict()
        self.snaffles = dict()
        self.my_score = 0
        self.my_magic = 0
        self.opp_score = 0
        self.opp_magic = 0
        self.target_score = 4
        self.target_snaffles_1 = dict()
        self.target_snaffles_2 = dict()
        self.all_units = dict()
    
    def __str__(self):
        s = f"Round {self.rounds}\n"
        # s += f"Me: {self.my_score} ({self.my_magic})\n"
        # s += f"Opp: {self.opp_score} ({self.opp_magic})\n"
        # s += f"My wizz1:\n {self.my_wizz1}\n"
        # s += f"My wizz2:\n {self.my_wizz2}\n"
        # s += f"Opp wizz1:\n {self.opp_wizz1}\n"
        # s += f"Opp wizz2:\n {self.opp_wizz2}\n"
        # n = len(self.snaffles)
        # s += f"Snaffles: {n}\n"
        # # for k, val in self.snaffles.items():
        # #     s += f"{val}\n"
        s += "Bludgers\n"
        for k, val in self.bludgers.items():
            s += f"{val}\n"
            
        # s += f"W1 {self.my_wizz1.role}:\n"
        # s += f"SP: {self.my_wizz1.sector_pref}\n"
        # s += f"AIM ID: {self.my_wizz1.aim_id}\n"
        # s += f"W2 {self.my_wizz2.role}\n"
        # s += f"SP: {self.my_wizz2.sector_pref}\n"
        # s += f"AIM ID: {self.my_wizz2.aim_id}\n"
        return s
    
    def initialize(self):
        self.rounds -= 1
        self.my_score, self.my_magic = [int(i) for i in input().split()]
        self.opp_score, self.opp_magic = [int(i) for i in input().split()]
        entities = int(input())
        for i in range(entities):
            entity_id, entity_type, x, y, vx, vy, state = input().split()
            uid = int(entity_id)
            x = int(x)
            y = int(y)
            vx = int(vx)
            vy = int(vy)
            state = int(state)
            if entity_type == "WIZARD":
                if not self.my_wizz1:
                    self.my_wizz1 = Wizzard(uid, x, y, vx, vy, state)
                else:
                    self.my_wizz2 = Wizzard(uid, x, y, vx, vy, state)
            elif entity_type == "OPPONENT_WIZARD":
                if not self.opp_wizz1:
                    self.opp_wizz1 = Wizzard(uid, x, y, vx, vy, state)
                    self.opp_wizzs[uid] = self.opp_wizz1
                else:
                    self.opp_wizz2 = Wizzard(uid, x, y, vx, vy, state)
                    self.opp_wizzs[uid] = self.opp_wizz2
            elif entity_type == "SNAFFLE":
                self.snaffles[uid] = Snaffle(uid, x, y, vx, vy, state)
            elif entity_type == "BLUDGER":
                self.bludgers[uid] = Bludger(uid, x, y, vx, vy, state)
        self.target_score = round((len(self.snaffles) / 2) + 0.5)
        print(f"Target SCORE: {self.target_score}", file=sys.stderr)
    
    def update(self):
        if self.my_wizz1 and self.my_wizz2 and self.my_wizz1.uid > self.my_wizz2.uid:
            self.my_wizz1, self.my_wizz2 = self.my_wizz2, self.my_wizz1
            print("My Wizzards UID SWAP!", file=sys.stderr)
        self.rounds -= 1
        self.my_score, self.my_magic = [int(i) for i in input().split()]
        self.opp_score, self.opp_magic = [int(i) for i in input().split()]
        entities = int(input())  # number of entities still in game
        current_snaffles = list()
        for i in range(entities):
            entity_id, entity_type, x, y, vx, vy, state = input().split()
            uid = int(entity_id)
            x = int(x)
            y = int(y)
            vx = int(vx)
            vy = int(vy)
            state = int(state)
            if entity_type == "WIZARD":
                self.update_mywizz(uid, x, y, vx, vy, state)
            elif entity_type == "OPPONENT_WIZARD":
                self.update_oppwiz(uid, x, y, vx, vy, state)
            elif entity_type == "SNAFFLE":
                self.snaffles[uid].update(x, y, vx, vy, state)
                current_snaffles.append(uid)
            elif entity_type == "BLUDGER":
                self.bludgers[uid].update(x, y, vx, vy, state)
        self.remove_snaffles(current_snaffles)
        if not self.my_wizz1.aim_id in self.snaffles.keys():
            self.my_wizz1.aim_id = None
        if not self.my_wizz2.aim_id in self.snaffles.keys():
            self.my_wizz2.aim_id = None

    def update_mywizz(self, uid, x, y, vx, vy, state):
        if self.my_wizz1.uid == uid:
            self.my_wizz1.update(x, y, vx, vy, state)
        elif self.my_wizz2.uid == uid:
            self.my_wizz2.update(x, y, vx, vy, state)
    
    def update_oppwiz(self, uid, x, y, vx, vy, state):
        if self.opp_wizz1.uid == uid:
            self.opp_wizz1.update(x, y, vx, vy, state)
        elif self.opp_wizz2.uid == uid:
            self.opp_wizz2.update(x, y, vx, vy, state)

    def remove_snaffles(self, inplay):
        to_rem = list()
        for key in self.snaffles.keys():
            if not key in inplay:
                to_rem.append(key)
        for key in to_rem:
            del self.snaffles[key]
    
    def strat_desirability(self):
        def next_fluent(x, y):
            for key, val in self.snaffles.items():
                if abs(self.enemy_goal - val.loc[0]) < abs(self.enemy_goal - x) and abs(x - val.loc[0]) < fluency_axis_dist and abs(y - val.loc[1]) < fluency_axis_dist:
                        return key
            return None
        
        def score_ascending(key, data, scb):
            ratio = 2 if (key=='w1' or key=='w2') else 1
            score = scb.copy()
            snaffs = sorted(self.snaffles.keys(), key=(lambda k: data[k][key]))
            print(f"Score \'{key}\': {snaffs}", file=sys.stderr)
            sc = 1
            for sn in snaffs:
                score[sn] += sc * ratio
                sc += 1
            return score

        def score_descending(key, data, score_board):
            score = score_board.copy()
            snaffs = sorted(self.snaffles.keys(), key=(lambda k: data[k][key]), reverse=True)
            print(f"Score \'{key}\': {snaffs}", file=sys.stderr)
            sc = 1
            for sn in snaffs:
                score[sn] += sc
                sc += 1
            return score
        
        def majority_score(data, score_board, close_to_mine, left, maj):
            right = maj - left
            left *= -1
            importancy_ratio = 5
            sc = score_board.copy()
            snaffs = sorted(self.snaffles.keys(), key=(lambda k: data[k][close_to_mine]))
            for k in snaffs:
                if data[k]['mci']:
                    if left ==0:
                        left += maj
                        importancy_ratio = 1
                    sc[k] += (left * importancy_ratio) 
                    left += 1
                else:
                    sc[k] += right
                    right += 1
            return sc
                
        if len(self.snaffles) == 1:
            a = next(iter(self.snaffles.keys()))
            self.my_wizz1.aim_id = a
            self.my_wizz2.aim_id = a
            return
        cluster_distance = 2500
        fluency_axis_dist = ((cluster_distance ** 2) / 2) ** 0.5
        data = dict()
        op_side = 0
        my_side = 0
        if distance(self.my_wizz1.loc, self.my_goal_loc) < distance(self.my_wizz2.loc, self.my_goal_loc):
            my_wizz_goal_d = distance(self.my_wizz1.loc, self.my_goal_loc)
            closer = 'w1'
        else:
            my_wizz_goal_d = distance(self.my_wizz2.loc, self.my_goal_loc)
            closer = 'w2'
        for key, val in self.snaffles.items():
            # Distance from my wizzs
            w1_dist, w1_rounds, w1_vec = self.my_wizz1.get_estimates(val)
            w2_dist, w2_rounds, w2_vec = self.my_wizz2.get_estimates(val)
            # w1 = w1_dist + w1_rounds * 150
            # w2 = w2_dist + w2_rounds * 150
            w1 = w1_rounds
            w2 = w2_rounds
            # Closest enemy
            op_w1_dist, op_w1_rounds, dump = self.opp_wizz1.get_estimates(val)
            op_w2_dist, op_w2_rounds, dump = self.opp_wizz2.get_estimates(val)
            op_score = min(op_w1_dist + op_w1_rounds * 150, op_w2_dist + op_w2_rounds * 150)
            # Distance from goals
            my_goal_dist = max(distance(self.my_goal_loc, val.loc), 1)
            op_goal_dist = distance(self.en_goal_loc, val.loc)
            if my_goal_dist < my_wizz_goal_d:
                my_side += 1
                # Majority Coeficient indicator
                mci = 1
            else:
                op_side += 1
                mci = 0
            goal_ratio = op_goal_dist / my_goal_dist
            # Clustering
            clust = 0
            for k2, v2 in self.snaffles.items():
                if v2.uid != val.uid:
                    if distance(v2.loc, val.loc) < cluster_distance:
                        clust += 1
            # Fluency
            fluency_1 = 0
            x = val.loc[0]
            y = val.loc[1]
            f = 1 if distance(self.my_wizz1.loc, self.my_goal_loc) >= my_goal_dist else 0
            while f:
                f = next_fluent(x, y)
                if f:
                    x = self.snaffles[f].loc[0]
                    y = self.snaffles[f].loc[1]
                    fluency_1 += 1
            fluency_2 = 0
            x = val.loc[0]
            y = val.loc[1]
            f = 1 if distance(self.my_wizz2.loc, self.my_goal_loc) >= my_goal_dist else 0
            while f:
                f = next_fluent(x, y)
                if f:
                    x = self.snaffles[f].loc[0]
                    y = self.snaffles[f].loc[1]
                    fluency_2 += 1
            
            data[key] = {'w1' : w1, 'w1_d' : w1_dist,'w1_vec' : w1_vec, 'w2' : w2, 'w2_d' : w2_dist,'w2_vec' : w2_vec, 'op': op_score, 'gr' : goal_ratio, 'cl' : clust, 'fl1': fluency_1, 'fl2': fluency_2, 'mci':mci}
        score = dict()
        for key in self.snaffles.keys():
            score[key] = 0
        score = score_descending('op', data, score)
        score = score_ascending('gr', data, score)
        score = score_descending('cl', data, score)
        w1_sc = score_descending('fl1', data, score)
        w2_sc = score_descending('fl2', data, score)
        w1_sc = score_ascending('w1', data, score)
        w2_sc = score_ascending('w2', data, score)
        n = self.target_score - self.my_score
        w1_sc = majority_score(data, w1_sc, closer, n - op_side, n)
        w2_sc = majority_score(data, w2_sc, closer, n - op_side, n)
        print(f"W1 SCORE: {w1_sc}", file=sys.stderr)
        print(f"W2 SCORE: {w2_sc}", file=sys.stderr) 
        w1_que = sorted(w1_sc.keys(), key=(lambda k: w1_sc[k]))
        self.select_target('w1', data, w1_sc, w1_que)
        w2_que = sorted(w2_sc.keys(), key=(lambda k: w2_sc[k]))
        self.select_target('w2', data, w2_sc, w2_que)
        
        # Check if swap needed
        if self.check_target_swap(data):
            self.select_target('w2', data, w2_sc, w2_que)
            self.select_target('w1', data, w1_sc, w1_que) 
        
        
        print(f"Wizz 1 aim: Snaffle {self.my_wizz1.aim_id}", file=sys.stderr)
        print(f"Wizz 2 aim: Snaffle {self.my_wizz2.aim_id }", file=sys.stderr)

    def select_target(self, wizz, data, score_board, priority):
        n = self.target_score - self.my_score
        if wizz == 'w1':
            other = 'w2'
            to_assign =  self.my_wizz1
            other_wizz = self.my_wizz2
        else:
            other = 'w1'
            to_assign =  self.my_wizz2
            other_wizz = self.my_wizz1
        
        best = priority[0]
        if best == other_wizz.aim_id:
            best = priority[1]
        best_d = data[best][wizz]
        for k in priority:
            if score_board[best] == score_board[k] and data[k][wizz] < data[best][wizz] and (k != other_wizz.aim_id or n == 1):
                best = k
                best_d = data[k][wizz]
            elif score_board[best] < score_board[k]:
                break
        to_assign.aim_id = best
        to_assign.aim_vec = data[best][f'{wizz}_vec']

    def check_target_swap(self, data):
        w1 = self.my_wizz1
        w2 = self.my_wizz2
        wd1 = data[w1.aim_id]['w1'] * 150 + data[w1.aim_id]['w1_d']
        wd2 = data[w2.aim_id]['w2'] * 150 + data[w2.aim_id]['w2_d']
        #Cross Distance
        wd1_2 = data[w2.aim_id]['w1']
        wd2_1 = data[w1.aim_id]['w2']
        if w1.aim_id == w2.aim_id:
            if wd2 > wd1 and wd1 < 1800:
                w2.aim_id = None
            elif wd2 < wd1 and wd2 < 1800:
                w1.aim_id = None
        elif wd2 > 1000 and wd1 > 1000 and (wd1_2 < wd2 or wd2_1 < wd1):
            print(f"Swapping Targets {w1.aim_id}<->{w2.aim_id}", file=sys.stderr)
            w1.aim_id = None
            w2.aim_id = None
            return True
        return False
    
    def turn(self):
        self.strat_desirability()
        self.action_wizz1()
        self.action_wizz2()


    def action_wizz1(self):
        if self.my_wizz1.state:
            self.throw_snaffle(self.my_wizz1, self.my_wizz2)
        else:
            magic, uid = self.check_incoming_snaffle(self.my_wizz2.magic_id)
            if len(self.snaffles) > 1:
                m = self.my_magic // 2
            else:
                m = self.my_magic
            if not magic:
                magic, uid = self.check_magic_score(self.my_wizz2.magic_id)
            self.my_wizz1.magic_id = uid
            if magic == 1:
                self.do_defensive_magic(self.my_wizz1, self.my_wizz2, uid, m)
            elif magic == 2:
                aim = get_aim(self.snaffles[uid].vec, (self.enemy_goal, 3750))
                nv = self.snaffles[uid].new_vector(aim, m * 15)
                col, w_uid, r, ang, cv = self.snaffles[uid].check_for_collision(self.opp_wizzs, nv, 15)
                if col:
                    print(f"CW! S{uid} & OW{w_uid} in {r}, {ang:.2f}", file = sys.stderr)
                self.my_magic -= m
                print(f"WINGARDIUM {uid} {aim[0]} {aim[1]} {m}")
            else:
                s_uid = self.my_wizz1.aim_id
                if s_uid:
                    self.my_wizz1.default_speech()
                    self.my_wizz1.move_to_object(self.snaffles[s_uid].loc, self.snaffles[s_uid].vec, self.bludgers, self.my_wizz2)
                else:
                    loc = self.my_wizz2.loc
                    loc = (loc[0] + (self.enemy_goal - loc[0]) // 2, loc[1])
                    self.my_wizz1.move_to_object(loc, self.my_wizz2.vec, self.bludgers, self.my_wizz2)
    
    def action_wizz2(self):
        if self.my_wizz2.state:
            self.throw_snaffle(self.my_wizz2, self.my_wizz1)
        else:
            magic, uid = self.check_incoming_snaffle(self.my_wizz1.magic_id)
            if not magic:
                magic, uid = self.check_magic_score(self.my_wizz1.magic_id)
            self.my_wizz2.magic_id = uid
            if magic == 1:
                self.do_defensive_magic(self.my_wizz2, self.my_wizz1, uid, self.my_magic)
            elif magic == 2:
                aim = get_aim(self.snaffles[uid].vec, (self.enemy_goal, 3750))
                m = self.my_magic
                nv = self.snaffles[uid].new_vector(aim, m * 15)
                col, w_uid, r, ang, cv = self.snaffles[uid].check_for_collision(self.opp_wizzs, nv, 15)
                if col:
                    print(f"CW! S{uid} & OW{w_uid} in {r}, {ang:.2f}", file = sys.stderr)
                self.my_magic -= m
                print(f"WINGARDIUM {uid} {aim[0]} {aim[1]} {m}") 
            else:
                s_uid = self.my_wizz2.aim_id
                if s_uid:
                    self.my_wizz2.default_speech()
                    self.my_wizz2.move_to_object(self.snaffles[s_uid].loc, self.snaffles[s_uid].vec, self.bludgers, self.my_wizz1)
                else:
                    loc = self.my_wizz1.loc
                    loc = (loc[0] + (self.enemy_goal - loc[0]) // 2, loc[1])
                    self.my_wizz2.move_to_object(loc, self.my_wizz1.vec, self.bludgers, self.my_wizz1)

    def check_incoming_snaffle(self, taken):
        if self.my_magic < 15 or self.rounds > 185:
            return False, None
        best = -1
        best_d = sys.maxsize
        g = (self.my_goal, 3750)
        d = 4000
        for key, val in self.snaffles.items():
            tmp = distance(g, val.loc)
            if tmp < 4000 and tmp < best_d and key != taken:
                best = val.uid
                best_d = tmp
        if best != -1:
            return 1, best
        else:
            return 0, None
    
    def check_magic_score(self, taken):
        if (self.my_magic < 35 and self.target_score - self.my_score > 1) or self.my_magic < 10:
            return 0, None
        for key, val in self.snaffles.items():
            if abs(val.loc[0] - self.enemy_goal) < 2000 and key != taken:
                return 2, val.uid
        return 0, None
    
    def do_defensive_magic(self, wizz, other_wizz, uid, magic):
        def to_sides():
            if self.snaffles[uid].loc[1] < 3750:
                if self.my_goal and sv[0] > 0:
                    aim = get_aim(sv, (12000, 0))
                elif not self.my_goal and sv[0] < 0:
                    aim = get_aim(sv, (4000, 0))
                else:
                    aim = get_aim(sv, (self.enemy_goal, 3750))
            else:
                if self.my_goal and sv[0] > 0:
                    aim = get_aim(sv, (12000, 7500))
                elif not self.my_goal and sv[0] < 0:
                    aim = get_aim(sv, (4000, 7500))
                else:
                    aim = get_aim(sv, (self.enemy_goal, 3750))
            return aim

        sv = self.snaffles[uid].vec
        if uid == wizz.aim_id:
            aim = get_aim(sv, wizz.loc, wizz.vec)
        elif uid == other_wizz.aim_id:
            aim = get_aim(sv, other_wizz.loc, other_wizz.vec)
        else:
            aim = to_sides()
        m = magic
        nv = self.snaffles[uid].new_vector(aim, m * 15)
        col, w_uid, r, ang, cv = self.snaffles[uid].check_for_collision(self.opp_wizzs, nv, 15)
        if col:
            print(f"CW! S{uid} & OW{w_uid} in {r}, {ang:.2f}", file = sys.stderr)
            aim = to_sides()
        self.my_magic -= m
        print(f"WINGARDIUM {uid} {aim[0]} {aim[1]} {m}")

    def throw_snaffle(self, wizz, other_wizz):
        def pass_snaffle(wizz, other_wizz, aim, uid, force=500):
            naim = pass_aim(wizz, other_wizz)
            nv = self.snaffles[uid].new_vector(naim, force)
            col, b_uid, r, ang, cv = self.snaffles[uid].check_for_collision(self.bludgers, nv, 5, force)
            if not col or (ang < math.pi / 2 and ang > -1 * math.pi / 2):
                col, w_uid, r, ang, cv = self.snaffles[uid].check_for_collision(self.opp_wizzs, nv, 5, 500)
                if not col or (ang < math.pi / 2 and ang > -1 * math.pi / 2):
                    wizz.talk = "PASSING"
                    return True, naim
            return False, aim
        
        def check_for_pass(wizz, other_wizz, aim, uid, reach):
            suc = False
            if distance(wizz.loc, other_wizz.loc) < reach:
                suc, aim = pass_snaffle(wizz, other_wizz, aim, uid)
            if suc:
                print(f"THROW {aim[0]} {aim[1]} {force} {wizz.talk}")
                return True
            else:
                return False

        def bounce_wall(wizz, uid):
            print("Checking for Bounce!", file=sys.stderr)
            def bounce(wizz, uid, bonds):
                aim = get_aim(multiply_vector(wizz.vec, 1/0.75), bonds)
                nv = self.snaffles[uid].new_vector(aim, 500)
                col, w_uid, r, ang, cv = self.snaffles[uid].check_for_collision(self.opp_wizzs, nv, 5, 500)
                if not col or (ang < math.pi / 2 and ang > -1 * math.pi / 2):
                    col, w_uid, r, ang, cv = self.snaffles[uid].check_for_collision(self.bludgers, nv, 5, 500)
                    if col and (ang > math.pi / 2 or ang < -1 * math.pi / 2):
                        return False, None
                return True, aim
            
            def upper(wizz, uid):
                bonds = (round(wizz.loc[0] + (self.enemy_goal - wizz.loc[0]) / 2), 0)
                return throw(wizz, uid, bonds, 500, 45)    
            
            def lower(wizz, uid):
                bonds = (round(wizz.loc[0] + (self.enemy_goal - wizz.loc[0]) / 2), 7500)
                return throw(wizz, uid, bonds, 500, 45)
                
            if wizz.loc[1] < 3750:
                suc, aim = upper(wizz, uid)
                if not suc:
                    suc, aim = lower(wizz, uid)
            else:
                suc, aim = lower(wizz, uid)
                if not suc:
                    suc, aim = upper(wizz, uid)
            if suc:
                wizz.talk = "LeBounce!"
                print(f"THROW {aim[0]} {aim[1]} {force} {wizz.talk}")
                return True
            else:
                return False
        
        def test_throw(aim, uid, force):
            rnds = 10
            nv = self.snaffles[uid].new_vector(aim, force)
            col, w_uid, r, ang,cv = self.snaffles[uid].check_for_collision(self.opp_wizzs, nv, rnds, force)
            if not col or (ang < math.pi / 2 and ang > -1 * math.pi / 2):
                col, w_uid, r, ang, cv = self.snaffles[uid].check_for_collision(self.bludgers, nv, rnds, force)
                if not col or (ang < math.pi / 2 and ang > -1 * math.pi / 2):
                    return True
            return False

        def throw(wizz, uid, towards, force, limit=None):
            bound = 90 if not limit else limit
            step = bound // 30
            if not step:
                step = 1
            default = get_aim(wizz.vec, towards)
            for i in range(0, bound // 2, step):
                aim = vector_rotate(default, math.radians(i))
                if test_throw(aim, uid, force):
                    return True, aim
                aim = vector_rotate(default, math.radians(-i))
                if test_throw(aim, uid, force):
                    return True, aim
            return False, None
            
        force = 500
        uid = wizz.uid_holding(self.snaffles)
        print(f"W2 holding S({uid})", file=sys.stderr)
        aim = get_aim(multiply_vector(wizz.vec, 1/0.75), (self.enemy_goal, 3750))
        nv = self.snaffles[uid].new_vector(aim, force)
        wizz.talk = "THROWING"
        if abs(wizz.loc[0] - self.enemy_goal) < 3350:
            t, target = throw(wizz, uid, (self.enemy_goal, 3750), force)
            if t:
                wizz.talk = "GNITOOHS"
                print(f"THROW {target[0]} {target[1]} {force} {wizz.talk}")
                return
        if abs(wizz.loc[0] - self.enemy_goal)  > abs(other_wizz.loc[0] - self.enemy_goal) and check_for_pass(wizz, other_wizz, aim, uid, 2600):
            return
        col, w_uid, r, ang, cv = self.snaffles[uid].check_for_collision(self.opp_wizzs, nv, 12, force)
        if col and (ang > math.pi / 2 or ang < -1 * math.pi / 2):
            # Check for other goal's locations
            print(f"CW! S{uid} & OW{w_uid} in {r}, {ang:.2f}", file = sys.stderr)
            if check_for_pass(wizz, other_wizz, aim, uid, 3200):
                return
            if bounce_wall(wizz, uid):
                return
        col, b_uid, r, ang, cv = self.snaffles[uid].check_for_collision(self.bludgers, nv, 12)
        if col and (ang > math.pi / 2 or ang < -1 * math.pi / 2):
            print(f"CW! S{uid} & BL{b_uid} in {r}, {ang:.2f}", file = sys.stderr)
            if check_for_pass(wizz, other_wizz, aim, uid, 3200):
                return
            if bounce_wall(wizz, uid):
                return
        print(f"Throw ORG: {aim}", file=sys.stderr)
        t, target = throw(wizz, uid, (self.enemy_goal, 3750), force, 360)
        if t:
            print(f"Throw RNG: {aim}", file=sys.stderr)
            wizz.talk = "MODNAR"
            print(f"THROW {target[0]} {target[1]} {force} {wizz.talk}")
        else:
            print(f"THROW {aim[0]} {aim[1]} {force} {wizz.talk}")

my_team_id = int(input())  # if 0 you need to score on the right of the map, if 1 you need to score on the left

game = Match(my_team_id)
init = True
# game loop
while True:
    if init:
        game.initialize()
        init = False
    else:
        game.update()
    print(game, file=sys.stderr)
    game.turn()
