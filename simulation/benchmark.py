"""A deterministic parameter sweep; includes an intentional failure case."""
import argparse
import json
from pathlib import Path
from .run import experiment

CASES = [
    ("Nominal", 1.5, .8, 1.),
    ("Heavy payload", 4., .8, 1.),
    ("Stronger pushes", 1.5, .8, 1.5),
    ("Slippery floor", 1.5, .35, 1.),
    ("Beyond stance limits", 4., .25, 2.5),
]


def meets_mission(metrics):
    return (not metrics["fell"] and metrics["peakTiltDeg"] < 3
            and metrics["heightRmseMm"] < 6 and metrics["maxDriftMm"] < 100
            and all(t is not None and t < 1 for t in metrics["recoverySeconds"]))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("public/sentry/benchmark.json"))
    args = parser.parse_args()
    rows = []
    for name, payload, friction, scale in CASES:
        for controller in ("whole_body", "joint_pd"):
            result = experiment(controller, payload, friction, scale)
            metrics = result["metrics"]
            rows.append({"case": name, "controller": controller, "config": result["config"],
                         "metrics": metrics, "meetsMission": meets_mission(metrics)})
            print(f"{name:22} {controller:12} tilt={metrics['rmsTiltDeg']:.3f} deg  fell={metrics['fell']}  mission={meets_mission(metrics)}", flush=True)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps({"criteria": "No fall; peak tilt <3 deg; height RMSE <6 mm; drift <100 mm; each recovery <1 s", "rows": rows}, indent=2), encoding="utf-8")


if __name__ == "__main__":
    main()
