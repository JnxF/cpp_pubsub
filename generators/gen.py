import string

ALPHABET = string.ascii_lowercase + string.digits + string.ascii_uppercase
for t in range(256):
    if chr(t) not in ALPHABET:
        ALPHABET += chr(t)
ALPHABET = list(ALPHABET)


def recursive_generation(letters, n, tot):
    if n == 0:
        for c in ALPHABET:
            letters[tot - 1] = c
            print("".join(letters), end="")
    else:
        for c in ALPHABET:
            letters[tot - 1 - n] = c
            recursive_generation(letters, n - 1, tot)


if __name__ == "__main__":
    n = 0
    while True:
        letters = [0 for _ in range(n + 1)]
        tot = n + 1
        recursive_generation(letters, n, tot)
        n += 1
