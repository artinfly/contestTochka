import json
from datetime import datetime

def parse_date(date_str):
    return datetime.strptime(date_str, "%Y-%m-%d").date()

def check_capacity(max_capacity, guests):
    events = []
    for guest in guests:
        events.append((parse_date(guest["check-in"]), 1))
        events.append((parse_date(guest["check-out"]), -1))
    events.sort()

    current = 0
    for _, delta in events:
        current += delta
        if current > max_capacity:
            return False
    return True

if __name__ == "__main__":
    max_capacity = int(input())
    n = int(input())
    guests = [json.loads(input()) for _ in range(n)]

    result = check_capacity(max_capacity, guests)
    print(result)
