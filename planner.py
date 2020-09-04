# -*- coding: utf-8 -*-
from buchi import mission_to_buchi
from product import ProdAut
from ts import distance, reach_waypoint
from discrete_plan import dijkstra_plan_networkX, dijkstra_plan_optimal, improve_plan_given_history, dijkstra_targets, dijkstra_targets_reverse
from networkx.drawing.nx_pydot import write_dot
from gurobipy import *
import time


class ltl_planner(object):
    def __init__(self, ts, hard_spec, soft_spec):
        buchi = mission_to_buchi(hard_spec, soft_spec)
        self.product = ProdAut(ts, buchi)
        self.Time = 0
        self.cur_pose = None
        self.trace = []  # record the regions to be visited
        self.traj = []  # record the full trajectory
        self.opt_log = []
        # record [(time, prefix, suffix, prefix_cost, suffix_cost, total_cost)]
        self.com_log = []
        # record [(time, no_messages)]
        self.contract_time = 0  # 其他机器人协助动作的大约执行时间
        self.ETA_current_collaboration = 0
        self.delay = False

    def optimal(self, beta=10, style='static'):
        self.beta = beta
        if style == 'static':
            # full graph construction
            self.product.graph['ts'].build_full()
            # write_dot(self.product.graph['ts'], "./ts.dot")
            self.product.build_full()
            self.run, plantime = dijkstra_plan_networkX(self.product, self.beta)
        elif style == 'ready':
            self.product.build_full()
            self.run, plantime = dijkstra_plan_networkX(self.product, self.beta)
        elif style == 'on-the-fly':
            # on-the-fly construction
            self.product.build_initial()
            self.product.build_accept()
            self.run, plantime = dijkstra_plan_optimal(self.product, self.beta)
        if self.run is None:
            print '---No valid has been found!---'
            print '---Check you FTS or task---'
            return
        # print '\n'
        print '------------------------------'
        print 'the prefix of plan **states**:'
        print [n for n in self.run.line]
        print 'the suffix of plan **states**:'
        print [n for n in self.run.loop]
        print '------------------------------'
        print 'the prefix of plan **aps**:'
        print [self.product.graph['ts'].node[n]['label'] for n in self.run.line]
        print 'the suffix of plan **aps**:'
        print [self.product.graph['ts'].node[n]['label'] for n in self.run.loop]
        # print '\n'
        print '------------------------------'
        # print 'the prefix of plan **actions**:'
        # print [n for n in self.run.pre_plan]
        # print 'the suffix of plan **actions**:'
        # print [n for n in self.run.suf_plan]
        self.opt_log.append((self.Time, self.run.pre_plan, self.run.suf_plan, self.run.precost, self.run.sufcost, self.run.totalcost))
        self.last_time = self.Time
        self.acc_change = 0
        self.index = 1
        self.segment = 'line'
        self.next_move = self.run.pre_plan[self.index]
        return plantime

    def find_next_move(self):
        # if self.delay and self.contract_time <= 0:
        #     return self.next_move
        if self.segment == 'line' and self.index < len(self.run.pre_plan)-2:
            # print 'index:', self.index
            # print 'pre_plan:', self.run.pre_plan
            # print 'line:', self.run.line
            self.trace.append(self.run.line[self.index])
            self.index += 1
            self.next_move = self.run.pre_plan[self.index]
        elif (self.segment == 'line') and ((self.index == len(self.run.pre_plan)-2) or (len(self.run.pre_plan) <= 2)):
            self.trace.append(self.run.line[self.index])
            self.index = 0
            self.segment = 'loop'
            self.next_move = self.run.suf_plan[self.index]
        elif self.segment == 'loop' and self.index < len(self.run.suf_plan)-2:
            self.trace.append(self.run.loop[self.index])
            self.index += 1
            self.segment = 'loop'
            self.next_move = self.run.suf_plan[self.index]
        elif (self.segment == 'loop') and ((self.index == len(self.run.suf_plan)-2) or (len(self.run.suf_plan) <= 2)):
            self.trace.append(self.run.loop[self.index])
            self.index = 0
            self.segment = 'loop'
            self.next_move = self.run.suf_plan[self.index]
        return self.next_move

    def update(self, object_name):
        MotionFts = self.product.graph['ts'].graph['region']
        cur_region = MotionFts.closest_node(self.cur_pose)
        sense_info = dict()
        sense_info['label'] = set([(cur_region, set([object_name, ]), set()), ])
        changes = MotionFts.update_after_region_change(sense_info, None)
        if changes:
            return True

    def replan(self):
        new_run = improve_plan_given_history(self.product, self.trace)
        if new_run and (new_run.pre_plan != self.run.pre_plan[self.index:-1]):
            self.run = new_run
            self.index = 1
            self.segment = 'line'
            self.next_move = self.run.pre_plan[self.index]
            print 'Plan adapted!'

    def cooperative_action_in_horizon(self, dep, horizon):
        s = 0
        Tm = 0
        Request = []
        init_run = self.run.prefix + self.run.suffix[1:]
        if self.segment == 'line':
            index = self.index - 1
        else:
            index = self.index + len(self.run.pre_plan) - 1
        # print 'prefix:', self.run.prefix
        # print 'suffix:', self.run.suffix
        # print 'init_run:', init_run
        while Tm < horizon and index + s < len(init_run) - 1:
            current_node = init_run[index + s]
            next_node = init_run[index + s + 1]
            # print 'current:', current_node, 'next:', next_node
            Tm = Tm + self.product.edges[current_node, next_node]['weight']
            # 0 represents 'ts' in next_node, and 0 represents region in next_node[0], 1 represents action
            reg = next_node[0][0]
            act = next_node[0][1]
            if not Request and act in dep:
                for action in dep[act]:
                    Request.append([action, reg, Tm])
            s += 1
        return Request

    def evaluate_request(self, request, dep, alpha):
        MAX = float(10000)
        reply = {}
        run = {}
        for req in request:
            act = req[0]
            reg = req[1]
            Tm = req[2]
            if self.contract_time <= 0 and not self.delay and not isinstance(self.next_move, basestring):
                if self.segment == 'line':
                    current_node = self.run.prefix[self.index - 1]
                    accept_node = self.run.prefix[-1]
                else:
                    if self.index == 0:
                        current_node = self.run.prefix[-1]
                    else:
                        current_node = self.run.suffix[self.index - 1]
                    accept_node = self.run.suffix[-1]
                sd = []
                sc = []
                for prod_node in self.product.nodes:
                    if reg == prod_node[0][0] and act == prod_node[0][1]:
                        sd.append(prod_node)
                    for col in dep:
                        if col == prod_node[0][1] or (prod_node[0][1] != act and prod_node[0][1] in dep[col]):
                            sc.append(prod_node)
                cost = 0
                index = self.index - 1
                if self.segment == 'line':
                    while index < len(self.run.prefix) - 1:
                        cost += self.product.edges[self.run.prefix[index], self.run.prefix[index + 1]]['weight']
                        index += 1
                else:
                    while index < len(self.run.suffix) - 1:
                        cost += self.product.edges[self.run.suffix[index], self.run.suffix[index + 1]]['weight']
                        index += 1
                for path1, cost1 in dijkstra_targets(self.product, current_node, sd, sc):
                    # print 'path1:', path1
                    for path2, cost2 in dijkstra_targets_reverse(self.product, accept_node, sd):
                        # print 'path2:', path2
                        if path1 and path2 and path1[-1] == path2[-1]:
                            cost3 = abs(cost1 - Tm) + alpha * (cost1 + cost2 - cost)
                            run[path1[-1]] = (path1 + list(reversed(path2))[1:], cost3, cost1)
                if run:
                    path, total_cost, pre_cost = min(run.values(), key=lambda p: p[1])
                    reply[act] = [path, True, pre_cost]
                else:
                    reply[act] = [[], False, MAX]
            else:
                reply[act] = [[], False, MAX]
        return reply

    def confirmation(self, request, Reply):
        start = time.time()
        aj = [key for key in Reply]
        d = [act[0] for act in request]
        confirm = {}
        t = {}
        b = {}
        Tm = {}
        for key in Reply:
            for req in request:
                t[req[0], key] = Reply[key][req[0]][2]
                if Reply[key][req[0]][1]:
                    b[req[0], key] = 1
                else:
                    b[req[0], key] = 0
                Tm[req[0]] = req[2]
        m = Model("confirm")
        c = m.addVars(d, aj, vtype=GRB.BINARY, name="c")
        tx = m.addVars(d, aj, name='tx')
        fm = m.addVar(name="fm")
        m.setObjective(fm, GRB.MINIMIZE)
        m.addGenConstrMax(fm, [tx[i, j] for i in d for j in aj], Tm[d[0]])
        m.addConstrs((c.prod(b, '*', j) <= 1 for j in aj), "constr1")
        m.addConstrs((c.prod(b, i, '*') == 1 for i in d), "constr2")
        m.addConstrs((c[i, j] * t[i, j] * b[i, j] - tx[i, j] == 0 for i in d for j in aj), "constr3")
        m.optimize()
        try:
            solution = m.getAttr('x', c)
        except GurobiError:
            return confirm, 0
        else:
            print 'solution:', solution
            has_solution = False
            for num in aj:
                conf = {}
                for act in d:
                    if solution[act, num] == 1:
                        conf[act] = [Reply[num][act][0], Reply[num][act][1], fm.getAttr('x')]
                        has_solution = True
                    else:
                        conf[act] = [[], False, float('inf')]
                confirm[num] = conf
            if has_solution:
                self.delay = False
                self.contract_time = fm.getAttr('x')
                return confirm, time.time() - start
            else:
                # self.contract_time = 0
                return confirm, 0

    def adapt_plan(self, confirm):
        for act in confirm:
            message = confirm[act]
            if message[1]:
                self.contract_time = message[2]
                new_path = message[0]
                if self.segment == 'line':
                    print 'old_prefix:', self.run.prefix
                    print 'current_node:', self.run.prefix[self.index - 1]
                    # print 'path:', new_path
                    if self.index == 1:
                        self.run.prefix = self.run.prefix[:self.index] + new_path
                    else:
                        self.run.prefix = self.run.prefix[:self.index - 1] + new_path
                    print 'new_prefix:', self.run.prefix
                else:
                    if self.index == 1:
                        self.run.suffix = self.run.suffix[:self.index] + new_path
                    else:
                        self.run.suffix = self.run.suffix[:self.index - 1] + new_path
                    # print 'suffix:', self.run.suffix
                self.run.plan_output(self.product)
                if self.segment == 'line':
                    self.next_move = self.run.pre_plan[self.index]
                else:
                    self.next_move = self.run.suf_plan[self.index]
            else:
                continue

    def delay_cooperation(self, DELAY, STEP):
        self.contract_time = 0
        self.delay = True









