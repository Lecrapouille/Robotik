#!/usr/bin/env python3
"""Build the edge list read by Robotik-Fly --edges.

Called from external/compilation after the Shiu repository is cloned.
Column names are those of model.py: Presynaptic_Index, Postsynaptic_Index,
"Excitatory x Connectivity". The completeness file is only checked: its row
order is the neuron index.

    python prepare_edges.py
    python prepare_edges.py Connectivity_783.parquet Completeness_783.csv out.csv
"""

import sys

import pandas as pd


def main() -> None:
    connectivity = sys.argv[1] if len(sys.argv) > 1 else "Connectivity_783.parquet"
    completeness = sys.argv[2] if len(sys.argv) > 2 else "Completeness_783.csv"
    output = sys.argv[3] if len(sys.argv) > 3 else "edges.csv"
    neurons = pd.read_csv(completeness, index_col=0)
    frame = pd.read_parquet(connectivity)
    edges = frame[["Presynaptic_Index", "Postsynaptic_Index", "Excitatory x Connectivity"]]
    edges.columns = ["pre", "post", "w"]
    edges.to_csv(output, index=False)
    print(f"{len(neurons)} neurons, {len(edges)} synapses -> {output}", file=sys.stderr)


if __name__ == "__main__":
    main()
