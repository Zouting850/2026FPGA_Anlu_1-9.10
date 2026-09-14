import io
import re
import sys

PIPE = "|"
BS = "\\"


def cells(line):
    s = line.strip()
    return re.split(r"(?<!\\)\|", s)


def main(path):
    src = io.open(path, encoding="utf-8").read().split("\n")
    issues = []
    i = 0
    while i < len(src):
        if src[i].strip().startswith(PIPE):
            blk = []
            j = i
            while j < len(src) and src[j].strip().startswith(PIPE):
                blk.append((j + 1, src[j]))
                j += 1
            ncols = len(cells(blk[0][1])) - 2
            for ln, txt in blk[1:]:
                body = txt.strip()
                if re.fullmatch(r"[|:\-\s]+", body):
                    continue
                if len(cells(txt)) - 2 != ncols:
                    issues.append((ln, ncols, len(cells(txt)) - 2))
            i = j
        else:
            i += 1
    print("table column mismatches (unescaped pipes only):", issues if issues else "none")
    return 0 if not issues else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1] if len(sys.argv) > 1 else "README.md"))
