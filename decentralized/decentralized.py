import velocity_obstacle.velocity_obstacle as velocity_obstacle
import nmpc.nmpc as nmpc
import argparse

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-m", "--mode", help="mode of obstacle avoidance; options: vo, or nmpc")
    parser.add_argument(
        "-f", "--filename", help="animation filename; eg: animation.gif")

    args = parser.parse_args()
    if args.mode == "vo":
        velocity_obstacle.simulate(args.filename)
    elif args.mode == "nmpc":
        nmpc.simulate(args.filename)
    else:
        print("Please choose the algorithm: vo or nmpc")
