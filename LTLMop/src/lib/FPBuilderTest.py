import FPBuild

def main():
    sensors = ["enemy", "ground_casualty"]
    actions = ["attack", "attack_over", "take_photo"]
    customs = ["attack_mode"]
    builder = FPBuild.FPBuilder("ResCap", "FPBuilder/output")
    builder.buildAll(sensors, actions, customs, add_write=False, init_clean=True)

if __name__ == "__main__":
    main()