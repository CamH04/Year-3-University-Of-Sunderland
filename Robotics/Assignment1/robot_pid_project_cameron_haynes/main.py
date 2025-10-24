import sys
from robot_pid.simulator import Simulator

# run simulaton, gracefully exit on keyboard input or exeption
def main():
    try:
        sim = Simulator()
        sim.run()
    except KeyboardInterrupt:
        print("\n\nSimulation interrupted by user.")
        sys.exit(0)
    except Exception as e:
        print(f"\nError occurred during simulation: {e}", file=sys.stderr)
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
