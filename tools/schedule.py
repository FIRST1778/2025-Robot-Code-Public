import json
from collections import defaultdict
with open("gpk.json", "r") as file:
    data = json.load(file)
partners = defaultdict(list)
for match_ in data:
    num = match_["match_number"]
    for color, details in match_["alliances"].items():
        teams = list(map(lambda s: int(s.removeprefix("frc")), details["team_keys"]))
        if 1778 in teams:
            for t in teams:
                partners[t].append(num)
del partners[1778]
for p in sorted(partners.keys()):
    print(p, " (", ", ".join(map(str, partners[p])), ")", sep='')
