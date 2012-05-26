#!/usr/bin/env python

import pstats

st = pstats.Stats('profile.out')
st.strip_dirs().sort_stats('cumulative').print_stats(25)
