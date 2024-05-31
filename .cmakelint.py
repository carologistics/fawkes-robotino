# ----------------------------
# Options affecting the linter
# ----------------------------
with section("lint"):
    # C0307 complains after indentation after formatting
    # C0301 is the 80 character limit, but we do not enforce it,
    #       if the formatter is happy, so are we.
    # R0912 and R0915 enforce the max number of statements and
    #       if-statements per scope
    disabled_codes = []  # ["C0111", "C0307", "R0912", "R0915", "C0301", "C0103", "C0303"]
