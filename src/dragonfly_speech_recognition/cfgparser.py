#!/usr/bin/env python
# -*- coding: utf-8 -*-
# ----------------------------------------------------------------------------------------------------

"""Grammars for the ContextFreeGrammarParser are built from production rules, corresponding to the Rule-class below.
This means that sentences can be generated (and auto-completed), according to this grammar.
Moreover, sentences can be parsed according to the same rules.

See https://www.tutorialspoint.com/automata_theory/context_free_grammar_introduction.htm and https://en.wikipedia.org/wiki/Context-free_grammar for an introduction to context free grammars.

If there is a rule "A -> one", then that means that to generate something according to rule A, the generated sentence is "one"
In this example "A" is the lname. lname stands for left name, as it's on the left of the arrow.
Sentences are produced and parsed from left to right.

There can be multiple lines in the grammar definition file with the same lname, which simply add ways to produce/parse sentences for that lname.

Rules can refer to each other via their lname.
If a rule A defines a way to start a sentence and refers to B, that means the completion of rule A is one via rule B.
For example, the grammar:
A -> go B
B -> forward
B -> backward
can generate the sentences "go forward" and "go backward". And thus parse these sentences as well.

Rules can also have variables that will be assigned to when a sentence is parsed.
For example, the line:

    VP["action": A] -> V_PLACE[A]

adds a rule for the lname VP, with a field called "action", which will be set to A.
The value for A is determined by a rule with lname V_PLACE, which will determine the value of A.

The rule

    V_PLACE["place"] -> place | put

applies when the text is "place" or "put".
When that is the case, the rule applies and the text "place" is filled in for A.
That means when the text "put" is typed, the variable "action" will be assigned the value "place".

The whole grammar has an entry point, or root rule, from which all the other rules are referred.
Each rule forms branch upon branch, together building a Tree.

When a sentence is parsed, a Tree is built. While this happens, the variables are collected.
When the Tree is completely parsed, the collected variables and their assignments are fetched from the Tree with the get_semantics-method.
This returns a string. However, this string represents a (nested) dictionary that maps a variable to a value.

Semantics describe what a sentence means. In this case, it describes what action to perform and with what to perform it.

The semantics are returned to whomever called CFGParser.parse(...), usually the REPL on console.py.
The REPL sends the semantics to the action_server, which grounds the semantics by implementing the actions.
"""

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


class Option:
    """An option is a continuation of a sentence of where there are multiple ways to continue the sentence.
    These choices in an Option are called called conjuncts."""
    def __init__(self, lsemantic = "", conjs = None):
        """Constructor of an Option
        :param lsemantic the name of the semantics that the option is the continuation of. E.g. if the lsemantic is some action, this option might be the object to perform that action with.
        :param conjs the choices in this option"""
        self.lsemantic = lsemantic
        if conjs:
            self.conjuncts = conjs
        else:
            self.conjuncts = []

    def __repr__(self):
        return "Option(lsemantic='{lsem}', conjs={c})".format(lsem=self.lsemantic, c=self.conjuncts)

    def __eq__(self, other):
        if isinstance(other, Option):
            return self.lsemantic == other.lsemantic and self.conjuncts == other.conjuncts

        return False

    @staticmethod
    def from_cfg_def(option_definition, left_semantics):
        """Parse text from the CFG definition into an Option and the choices it is composed of. """
        opt_strs = option_definition.split("|")

        for opt_str in opt_strs:
            opt_str = opt_str.strip()

            opt = Option(left_semantics)

            while opt_str:
                (rname, rsem, opt_str) = parse_next_atom(opt_str)
                is_variable = rname[0].isupper()
                opt.conjuncts += [Conjunct(rname, rsem, is_variable)]

            yield opt

    def pretty_print(self, level=0):
        # print self, level
        tabs = level*"    "
        ret = "\n"
        ret += tabs + "Option(lsemantic='{lsem}', conjs=[".format(lsem=self.lsemantic)
        for conj in self.conjuncts:
            #ret += "\n"
            #ret += tabs + "    " + "{c},".format(c=conj)
            ret += " "
            ret += conj.pretty_print()
        ret += "])"
        return ret

    def graphviz_id(self):
        return "Option '{lsem}'".format(lsem=self.lsemantic).replace('"', '').replace(":","")

    def to_graphviz(self, graph):
        for conj in self.conjuncts:
            graph.edge(self.graphviz_id(), conj.graphviz_id())
            conj.to_graphviz(graph)
# ----------------------------------------------------------------------------------------------------

class Conjunct:
    """"A Conjunct is a placeholder in the parse-tree, which can be filled in by an Option or a word"""

    def __init__(self, name, rsemantic="", is_variable=False):
        """:param name the word or variable
        :param rsemantic what option is the Conjunct part of
        :param is_variable is the conjunct variable or terminal?"""
        self.name = name
        self.rsemantic = rsemantic
        self.is_variable = is_variable

    def __repr__(self):
        return "Conjunct(name='{name}', rsemantic='{r}', is_variable={v})".format(name=self.name, r=self.rsemantic, v=self.is_variable)

    def __eq__(self, other):
        if isinstance(other, Conjunct):
            return self.name == other.name and self.rsemantic == other.rsemantic and self.is_variable == other.is_variable
        return False

    def pretty_print(self, level=0):
        if self.is_variable or "$" in self.name:
            prefix = self.rsemantic + "="
            return self.rsemantic + "=" + self.name# + str(self)
        else:
            return bcolors.OKGREEN + self.name + bcolors.ENDC# + str(self)

    def graphviz_id(self):
        return "Conjunct {name}".format(name=self.name)

    def to_graphviz(self, graph):
        graph.node(self.graphviz_id())

# ----------------------------------------------------------------------------------------------------

class Rule:

    def __init__(self, lname, options=None):
        self.lname = lname
        self.options = options if options else []

    def __repr__(self):
        return "Rule(lname='{lname}', options={opts})".format(lname=self.lname, opts=self.options)

    def __eq__(self, other):
        if isinstance(other, Rule):
            return self.lname == other.lname and self.options == other.options

        return False

    @staticmethod
    def from_cfg_def(s):
        tmp = s.split(" -> ")
        if len(tmp) != 2:
            raise Exception

        (lname, lsem, outstr) = parse_next_atom(tmp[0].strip())

        rule = Rule(lname)

        rule.options = list(Option.from_cfg_def(tmp[1], lsem))

        return rule

    def pretty_print(self, level=0):
        tabs = (level) * '    '
        ret = ""
        ret += tabs + self.lname
        for option in self.options:
            ret += option.pretty_print(level=level+1)
        return ret

    def graphviz_id(self):
        return "Rule {lname}".format(lname=self.lname)

    def to_graphviz(self, graph):
        for opt in self.options:
            graph.edge(self.graphviz_id(), opt.graphviz_id())
            opt.to_graphviz(graph)
# ----------------------------------------------------------------------------------------------------

class Tree:

    def __init__(self, option):
        self.option = option
        self.subtrees = [None for c in self.option.conjuncts]
        self.parent = None
        self.parent_idx = 0

    def next(self, idx):
        if idx + 1 < len(self.option.conjuncts):
            return (self, idx + 1)
        else:
            if self.parent:
                return self.parent.next(self.parent_idx)
            else:
                return (None, 0)

    def add_subtree(self, idx, tree):
        tree.parent = self
        tree.parent_idx = idx
        self.subtrees[idx] = tree
        return tree

    def __repr__(self):
        # TODO: Make this print like a tree
        return str(zip(self.option.conjuncts, self.subtrees))

    def pretty_print(self, level=0):
        # print self, level
        #tabs = (level-1)*'    ' + "│   ├───"
        tabs = (level)*'    ' + "└───"
        #tabs = "\t" * level #
        ret = "" #"#tabs + self.option.pretty_print(level=level)
        for conjunct, subtree in zip(self.option.conjuncts, self.subtrees):
            ret += tabs + conjunct.pretty_print() + "\n"
            if hasattr(subtree, "pretty_print"):
                ret += subtree.pretty_print(level=level+1)
                # ret += "\n"

        return ret

# ----------------------------------------------------------------------------------------------------

def parse_next_atom(s):
    """
    Returns (name, semantics, remaining_str)
    For example for "VP[X, Y] foo bar" it returns:
         ("VP", "X, Y", "foo bar")
    :param s:
    :return: Tuple with the rule's lname, the variables involved and the remaining text: ("VP", "X, Y", "foo bar")
    """
    s = s.strip()

    for i in range(0, len(s)):
        c = s[i]
        if c == ' ':
            return (s[:i], "", s[i:].strip())
        elif c == '[':
            j = s.find("]", i)
            if j < 0:
                raise Exception
            return (s[:i], s[i+1:j], s[j+1:].strip())

    return (s, "", "")

# ----------------------------------------------------------------------------------------------------

class CFGParser:

    def __init__(self):
        self.rules = {}
        self.functions = {}

    @staticmethod
    def fromfile(filename):
        parser = CFGParser()
        with open(filename) as f:
            for line in f:
                line = line.strip()
                if line == "" or line[0] == '#':
                    continue
                parser.add_rule(line)
        return parser

    def add_rule(self, s):
        rule = Rule.from_cfg_def(s)

        # See if a rule with this lname already exists. If not, add it
        if rule.lname in self.rules:
            original_rule = self.rules[rule.lname]
            original_rule.options += rule.options
        else:
            self.rules[rule.lname] = rule

    def set_function(self, name, func):
        self.functions[name] = func

    def get_semantics(self, tree):
        """Get the semantics of a tree.
        This means that variables are unified with their values, which may be recursively gotten from the tree's subtrees. """
        semantics = tree.option.lsemantic
        for i in range(0, len(tree.subtrees)):
            conj = tree.option.conjuncts[i]
            subtree = tree.subtrees[i]

            if subtree:
                child_semantics = self.get_semantics(subtree)
                semantics = semantics.replace(conj.rsemantic, child_semantics)

        return semantics

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    def parse(self, target, words, debug=False):
        if not target in self.rules:
            return False

        rule = self.rules[target]

        for opt in rule.options:
            T = Tree(opt)
            if self._parse((T, 0), words) != False:
                if debug:
                    print T.pretty_print()
                # Simply take the first tree that successfully parses
                return self.get_semantics(T)

        return False

    def _parse(self, TIdx, words):
        (T, idx) = TIdx

        if not T:
            return words == []

        if not words:
            return False

        conj = T.option.conjuncts[idx]

        if conj.is_variable:
            if not conj.name in self.rules:
                return False
            options = self.rules[conj.name].options

        elif conj.name[0] == "$":
            func_name = conj.name[1:]
            if not func_name in self.functions:
                return False
            options = self.functions[func_name](words)

        else:
            if conj.name == words[0]:
                return self._parse(T.next(idx), words[1:])
            else:
                return False

        for opt in options:
            subtree = T.add_subtree(idx, Tree(opt))
            ret = self._parse((subtree, 0), words)
            if ret:
                return ret

        return False

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    def next_word(self, target, words):
        if not target in self.rules:
            return False

        rule = self.rules[target]

        next_words = []
        for opt in rule.options:
            next_words += self._next_word((Tree(opt), 0), words)

        return next_words

    def _next_word(self, TIdx, words):
        (T, idx) = TIdx

        if not T:
            return []

        conj = T.option.conjuncts[idx]

        if conj.is_variable:
            if not conj.name in self.rules:
                return []
            options = self.rules[conj.name].options

        elif conj.name[0] == "$":
            func_name = conj.name[1:]
            if not self.has_completion_function(func_name):
                return False
            options = self.get_completion_function(func_name)(words)

        else:
            if words == []:
                return [conj.name]
            elif conj.name == words[0]:
                return self._next_word(T.next(idx), words[1:])
            else:
                return []

        next_words = []
        for opt in options:
            subtree = T.add_subtree(idx, Tree(opt))
            next_words += self._next_word((subtree, 0), words)

        return next_words

    def has_completion_function(self, func_name):
        return func_name in self.functions

    def get_completion_function(self, func_name):
        return self.functions[func_name]

    def graphviz_id(self):
        return "CFGParser"

    def to_graphviz(self, graph):
        for name, rule in self.rules.iteritems():
            graph.edge(self.graphviz_id(), rule.graphviz_id())
            rule.to_graphviz(graph)

    def visualize_options(self, graph, target_rule, previous_words=None, depth=2):
        previous_words = [] if not previous_words else previous_words

        import itertools
        colors = itertools.cycle(["blue", "green", "red", "cyan", "magenta", "black", "purple", "orange"])

        if previous_words:
            previous_word = previous_words[-1]
        else:
            previous_word = target_rule

        graph.node(previous_word)
        next_words = set(self.next_word(target_rule, previous_words))

        if next_words and depth:
            # print next_words

            for next_word in next_words:
                graph.edge(previous_word, next_word, color=colors.next())
                self.visualize_options(graph, target_rule, previous_words+[next_word], depth=depth-1)


class Visualizer(object):
    def __init__(self, grammarfile):
        self.parser = CFGParser.fromfile(grammarfile)

    def get_completion_function(self, name):
        return lambda x: [Option(name, [Conjunct(name.upper())])]

    def test(self, rule, depth):
        import graphviz
        self.parser.has_completion_function = lambda func_name: True
        self.parser.get_completion_function = self.get_completion_function

        g = graphviz.Digraph(strict=True)
        self.parser.visualize_options(g, rule, depth=int(depth))
        g.render('options', view=True)

if __name__ == "__main__":
    import sys

    grammar_file = sys.argv[1]
    rule = sys.argv[2]
    depth = int(sys.argv[3])

    tester = Visualizer(grammar_file)
    tester.test(rule, depth=depth)

