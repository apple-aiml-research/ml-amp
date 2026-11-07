//
// For licensing see accompanying LICENSE file.
// Copyright (C) 2026 Apple Inc. All Rights Reserved.
//
const AMP_VIDEOS = {
  "categories": [],
  "galleries": {
    "benchmarks": {
      "categories": [
        {
          "name": "LIBERO-PRO",
          "videos": [
            {
              "file": "videos/benchmarks/02_LIBERO-PRO/01_bowl-in-drawer.mp4",
              "thumb": "thumbnails/benchmarks/02_LIBERO-PRO/01_bowl-in-drawer.webp",
              "title": "Bowl in Drawer",
              "caption": "",
              "prompt": "Open the top layer of the drawer and put the bowl inside"
            },
            {
              "file": "videos/benchmarks/02_LIBERO-PRO/04_wine-bottle-on-plate.mp4",
              "thumb": "thumbnails/benchmarks/02_LIBERO-PRO/04_wine-bottle-on-plate.webp",
              "title": "Wine Bottle on Plate",
              "caption": "",
              "prompt": "Put the wine bottle on the plate"
            },
            {
              "file": "videos/benchmarks/02_LIBERO-PRO/05_cream-cheese-on-rack.mp4",
              "thumb": "thumbnails/benchmarks/02_LIBERO-PRO/05_cream-cheese-on-rack.webp",
              "title": "Cream Cheese on Rack",
              "caption": "",
              "prompt": "Put the cream cheese on the rack"
            },
            {
              "file": "videos/benchmarks/02_LIBERO-PRO/06_cream-cheese-in-drawer.mp4",
              "thumb": "thumbnails/benchmarks/02_LIBERO-PRO/06_cream-cheese-in-drawer.webp",
              "title": "Cream Cheese in Drawer",
              "caption": "",
              "prompt": "Open the top layer of the drawer and put the cream cheese inside"
            },
            {
              "file": "videos/benchmarks/02_LIBERO-PRO/07_orange-juice-in-basket.mp4",
              "thumb": "thumbnails/benchmarks/02_LIBERO-PRO/07_orange-juice-in-basket.webp",
              "title": "Orange Juice in Basket",
              "caption": "",
              "prompt": "Pick the orange juice and place it in the basket"
            },
            {
              "file": "videos/benchmarks/02_LIBERO-PRO/08_wine-bottle-on-drawer.mp4",
              "thumb": "thumbnails/benchmarks/02_LIBERO-PRO/08_wine-bottle-on-drawer.webp",
              "title": "Wine Bottle on Drawer",
              "caption": "",
              "prompt": "Put the wine bottle on the top of the drawer"
            }
          ]
        },
        {
          "name": "RoboDojo",
          "videos": [
            {
              "file": "videos/benchmarks/02_RoboDojo/01_build-tower.mp4",
              "thumb": "thumbnails/benchmarks/02_RoboDojo/01_build-tower.webp",
              "title": "Build a Tower",
              "caption": "",
              "prompt": "Build a tower using the wooden blocks and wooden boards."
            },
            {
              "file": "videos/benchmarks/02_RoboDojo/03_declare-kong.mp4",
              "thumb": "thumbnails/benchmarks/02_RoboDojo/03_declare-kong.webp",
              "title": "Declare a Kong",
              "caption": "",
              "prompt": "Wait for the opponent to discard a tile, then declare a kong with the matching tiles."
            },
            {
              "file": "videos/benchmarks/02_RoboDojo/05_pour-by-language.mp4",
              "thumb": "thumbnails/benchmarks/02_RoboDojo/05_pour-by-language.webp",
              "title": "Pour by Language",
              "caption": "",
              "prompt": "Pour the liquid from the first turquoise bottle into the first black bowl, from the second violet bottle into the second white bowl, and from the third red bottle into the third brown bowl. Then reset the robot arm."
            },
            {
              "file": "videos/benchmarks/02_RoboDojo/07_hang-mugs.mp4",
              "thumb": "thumbnails/benchmarks/02_RoboDojo/07_hang-mugs.webp",
              "title": "Hang Mugs",
              "caption": "",
              "prompt": "Hang all the mugs on the mug rack."
            },
            {
              "file": "videos/benchmarks/02_RoboDojo/09_pick-by-image.mp4",
              "thumb": "thumbnails/benchmarks/02_RoboDojo/09_pick-by-image.webp",
              "title": "Pick from a Conveyor by Image",
              "caption": "",
              "prompt": "Lift the basket more than 8 cm, identify the target object on the conveyor according to the image on the board, pick it up, and place it into the basket."
            },
            {
              "file": "videos/benchmarks/02_RoboDojo/10_fold-clothes.mp4",
              "thumb": "thumbnails/benchmarks/02_RoboDojo/10_fold-clothes.webp",
              "title": "Fold Clothes",
              "caption": "",
              "prompt": "Fold the clothes neatly."
            },
            {
              "file": "videos/benchmarks/02_RoboDojo/11_organize-table.mp4",
              "thumb": "thumbnails/benchmarks/02_RoboDojo/11_organize-table.webp",
              "title": "Organize a Table",
              "caption": "",
              "prompt": "Place the alarm clock on the drawer, put the figurine on the stand, place the mouse on the mouse pad, and push the keyboard into the frame."
            },
            {
              "file": "videos/benchmarks/02_RoboDojo/12_make-toast.mp4",
              "thumb": "thumbnails/benchmarks/02_RoboDojo/12_make-toast.webp",
              "title": "Make Toast",
              "caption": "",
              "prompt": "Pick up two slices of bread, place them into the toaster, and press the lever down."
            },
            {
              "file": "videos/benchmarks/02_RoboDojo/15_store-laptop.mp4",
              "thumb": "thumbnails/benchmarks/02_RoboDojo/15_store-laptop.webp",
              "title": "Store a Laptop and Headphones",
              "caption": "",
              "prompt": "Hang the headphones on the headphone stand, close the laptop, then place it into the vertical laptop stand."
            },
            {
              "file": "videos/benchmarks/02_RoboDojo/17_bottles-in-dustbin.mp4",
              "thumb": "thumbnails/benchmarks/02_RoboDojo/17_bottles-in-dustbin.webp",
              "title": "Put Bottles into a Dustbin",
              "caption": "",
              "prompt": "Pick up the bottles and throw them into the dustbin, using handover when needed."
            }
          ]
        },
        {
          "name": "Robosuite",
          "videos": [
            {
              "file": "videos/benchmarks/03_robosuite/05_two-arm-lift.mp4",
              "thumb": "thumbnails/benchmarks/03_robosuite/05_two-arm-lift.webp",
              "title": "Two-Arm Lift",
              "caption": "",
              "prompt": "The two arms should coordinate to lift a pot. Arm 0 should grasp handle 0, and Arm 1 should grasp handle 1. Then both arms should lift the pot up at the same height."
            },
            {
              "file": "videos/benchmarks/03_robosuite/01_cube-restack.mp4",
              "thumb": "thumbnails/benchmarks/03_robosuite/01_cube-restack.webp",
              "title": "Restack the Cube",
              "caption": "",
              "prompt": "Gently place the red cube on top of the green cube and then open the gripper. Nothing should be dropped from a height."
            },
            {
              "file": "videos/benchmarks/03_robosuite/02_nut-assembly.mp4",
              "thumb": "thumbnails/benchmarks/03_robosuite/02_nut-assembly.webp",
              "title": "Nut Assembly",
              "caption": "",
              "prompt": "Grasp and insert the brown square nut onto the brown square block."
            },
            {
              "file": "videos/benchmarks/03_robosuite/03_spill-wipe.mp4",
              "thumb": "thumbnails/benchmarks/03_robosuite/03_spill-wipe.webp",
              "title": "Wipe the Spill",
              "caption": "",
              "prompt": "Wipe up the 'brown spill'."
            },
            {
              "file": "videos/benchmarks/03_robosuite/06_cube-lifting.mp4",
              "thumb": "thumbnails/benchmarks/03_robosuite/06_cube-lifting.webp",
              "title": "Lift the Cube",
              "caption": "",
              "prompt": "Pick up the red cube and lift it."
            },
            {
              "file": "videos/benchmarks/03_robosuite/04_two-arm-handover.mp4",
              "thumb": "thumbnails/benchmarks/03_robosuite/04_two-arm-handover.webp",
              "title": "Two-Arm Handover",
              "caption": "",
              "prompt": "Arm 0 should pick up the hammer, lift it, and hand it over to Arm 1. Arm 1 should then grasp the hammer handle (not hammer head). The z-value of hammer must be between 0.15 and 0.20 during the handover to count as a success."
            }
          ]
        },
        {
          "name": "BEHAVIOR-1K",
          "videos": [
            {
              "file": "videos/benchmarks/04_BEHAVIOR-1K/01_pick-up-radio.mp4",
              "thumb": "thumbnails/benchmarks/04_BEHAVIOR-1K/01_pick-up-radio.webp",
              "title": "Pick Up the Radio",
              "caption": "",
              "prompt": "Pick up the red radio on the table."
            },
            {
              "file": "videos/benchmarks/04_BEHAVIOR-1K/02_pick-up-soda-can.mp4",
              "thumb": "thumbnails/benchmarks/04_BEHAVIOR-1K/02_pick-up-soda-can.webp",
              "title": "Pick Up the Soda Can",
              "caption": "",
              "prompt": "Pick up the blue can of soda on the floor."
            }
          ]
        }
      ]
    },
    "gallery": {
      "categories": [
        {
          "name": "Real (zero-shot)",
          "videos": [
            {
              "file": "videos/gallery/01_real/put-keys-in-cup.mp4",
              "thumb": "thumbnails/gallery/01_real/put-keys-in-cup.webp",
              "card": "videos/gallery/01_real/put-keys-in-cup_card.mp4",
              "title": "Put keys in cup",
              "caption": "",
              "prompt": "Put my keys in the cup",
              "emergent": "Searching for hidden objects"
            },
            {
              "file": "videos/gallery/01_real/sort-corner-brackets-by-weight.mp4",
              "thumb": "thumbnails/gallery/01_real/sort-corner-brackets-by-weight.webp",
              "card": "videos/gallery/01_real/sort-corner-brackets-by-weight_card.mp4",
              "title": "Sort corner brackets by weight",
              "caption": "",
              "prompt": "Sort the corner brackets by weight from heaviest to lightest, placing them from right to left.",
              "emergent": "Using a balance scale as a measuring instrument"
            },
            {
              "file": "videos/gallery/01_real/checkmate-then-put-pieces-away.mp4",
              "thumb": "thumbnails/gallery/01_real/checkmate-then-put-pieces-away.webp",
              "card": "videos/gallery/01_real/checkmate-then-put-pieces-away_card.mp4",
              "title": "Checkmate as black, then put the pieces away",
              "caption": "",
              "prompt": "Get checkmate as black. Then, lift the white king and drop it on its side, on the same square. Then, open the drawers, and put the pieces in the correct drawers. Make sure the pieces lie flat. Then, close the drawers."
            },
            {
              "file": "videos/gallery/01_real/bowl-in-cabinet.mp4",
              "thumb": "thumbnails/gallery/01_real/bowl-in-cabinet.webp",
              "card": "videos/gallery/01_real/bowl-in-cabinet_card.mp4",
              "title": "Put bowl in cabinet",
              "caption": "",
              "prompt": "Put the small white bowl in the shelf above the sink, on the left side. Then close the door."
            },
            {
              "file": "videos/gallery/01_real/set-the-table-neatly.mp4",
              "thumb": "thumbnails/gallery/01_real/set-the-table-neatly.webp",
              "card": "videos/gallery/01_real/set-the-table-neatly_card.mp4",
              "title": "Set the table neatly",
              "caption": "",
              "prompt": "Set the table neatly."
            },
            {
              "file": "videos/gallery/01_real/hide-pieces-under-cups.mp4",
              "thumb": "thumbnails/gallery/01_real/hide-pieces-under-cups.webp",
              "card": "videos/gallery/01_real/hide-pieces-under-cups_card.mp4",
              "title": "Hide pieces under cups, then uncover the knight",
              "caption": "",
              "prompt": "Hide the knight, the queen, and the pawn under the cups. Then, uncover the knight."
            },
            {
              "file": "videos/gallery/01_real/spoon-as-tool.mp4",
              "thumb": "thumbnails/gallery/01_real/spoon-as-tool.webp",
              "card": "videos/gallery/01_real/spoon-as-tool_card.mp4",
              "title": "Put mustard in bowl",
              "caption": "",
              "prompt": "Put mustard in the bowl",
              "emergent": "Using objects as tools"
            },
            {
              "file": "videos/gallery/01_real/shelf-the-book.mp4",
              "thumb": "thumbnails/gallery/01_real/shelf-the-book.webp",
              "card": "videos/gallery/01_real/shelf-the-book_card.mp4",
              "title": "Shelve the book",
              "caption": "",
              "prompt": "Shelve the book",
              "emergent": "Pushing to enable grasping"
            },
            {
              "file": "videos/gallery/01_real/win-the-game-as-black.mp4",
              "thumb": "thumbnails/gallery/01_real/win-the-game-as-black.webp",
              "card": "videos/gallery/01_real/win-the-game-as-black_card.mp4",
              "title": "Win the game as black",
              "caption": "",
              "prompt": "Win the game as black, then put the pieces in the correct drawers, and close the drawers."
            },
            {
              "file": "videos/gallery/01_real/sort-nine-cubes.mp4",
              "thumb": "thumbnails/gallery/01_real/sort-nine-cubes.webp",
              "card": "videos/gallery/01_real/sort-nine-cubes_card.mp4",
              "title": "Sort nine cubes",
              "caption": "",
              "prompt": "Sort the 9 cubes by color: red cubes in one cluster, blue cubes in another, kept apart.",
              "emergent": "Pushing to enable grasping"
            },
            {
              "file": "videos/gallery/01_real/nest-cups-in-largest-cup.mp4",
              "thumb": "thumbnails/gallery/01_real/nest-cups-in-largest-cup.webp",
              "card": "videos/gallery/01_real/nest-cups-in-largest-cup_card.mp4",
              "title": "Nest cups in largest cup",
              "caption": "",
              "prompt": "Nest the cups inside the largest cup"
            },
            {
              "file": "videos/gallery/01_real/put-cups-in-drawer.mp4",
              "thumb": "thumbnails/gallery/01_real/put-cups-in-drawer.webp",
              "card": "videos/gallery/01_real/put-cups-in-drawer_card.mp4",
              "title": "Put cups in drawer",
              "caption": "",
              "prompt": "Put all the cups in the drawer.",
              "emergent": "Correcting calibration errors"
            },
            {
              "file": "videos/gallery/01_real/put-book-in-bin.mp4",
              "thumb": "thumbnails/gallery/01_real/put-book-in-bin.webp",
              "card": "videos/gallery/01_real/put-book-in-bin_card.mp4",
              "title": "Put book in bin",
              "caption": "",
              "prompt": "Place the book in the bin.",
              "emergent": "Pushing to enable grasping"
            },
            {
              "file": "videos/gallery/01_real/put-yellow-block-in-bowl.mp4",
              "thumb": "thumbnails/gallery/01_real/put-yellow-block-in-bowl.webp",
              "card": "videos/gallery/01_real/put-yellow-block-in-bowl_card.mp4",
              "title": "Put yellow block in bowl",
              "caption": "",
              "prompt": "Put the yellow block in the bowl.",
              "emergent": "Searching for hidden objects"
            },
            {
              "file": "videos/gallery/01_real/set-an-elegant-table-for-two.mp4",
              "thumb": "thumbnails/gallery/01_real/set-an-elegant-table-for-two.webp",
              "card": "videos/gallery/01_real/set-an-elegant-table-for-two_card.mp4",
              "title": "Set an elegant table for two",
              "caption": "",
              "prompt": "Set an elegant table for two"
            },
            {
              "file": "videos/gallery/01_real/push-bowl-off-bin.mp4",
              "thumb": "thumbnails/gallery/01_real/push-bowl-off-bin.webp",
              "card": "videos/gallery/01_real/push-bowl-off-bin_card.mp4",
              "title": "Put bowl on plate",
              "caption": "",
              "prompt": "Put the bowl on the plate",
              "emergent": "Moving the goal instead of the object"
            },
            {
              "file": "videos/gallery/01_real/overhang-bowl-to-grasp.mp4",
              "thumb": "thumbnails/gallery/01_real/overhang-bowl-to-grasp.webp",
              "card": "videos/gallery/01_real/overhang-bowl-to-grasp_card.mp4",
              "title": "Put bowl on plate",
              "caption": "",
              "prompt": "Put the bowl on the plate",
              "emergent": "Pushing to enable grasping"
            },
            {
              "file": "videos/gallery/01_real/table1-put-the-rubiks-cube-to-the-left-of-the-bowl.mp4",
              "thumb": "thumbnails/gallery/01_real/table1-put-the-rubiks-cube-to-the-left-of-the-bowl.webp",
              "card": "videos/gallery/01_real/table1-put-the-rubiks-cube-to-the-left-of-the-bowl_card.mp4",
              "title": "Put Rubik's cube left of bowl",
              "caption": "",
              "prompt": "Put the rubiks cube to the left of the bowl"
            },
            {
              "file": "videos/gallery/01_real/table1-put-the-cube-and-the-banana-in-the-bowl.mp4",
              "thumb": "thumbnails/gallery/01_real/table1-put-the-cube-and-the-banana-in-the-bowl.webp",
              "card": "videos/gallery/01_real/table1-put-the-cube-and-the-banana-in-the-bowl_card.mp4",
              "title": "Put cube and banana in bowl",
              "caption": "",
              "prompt": "Put the cube and the banana in the bowl"
            },
            {
              "file": "videos/gallery/01_real/table1-put-all-the-red-things-in-the-grey-bin.mp4",
              "thumb": "thumbnails/gallery/01_real/table1-put-all-the-red-things-in-the-grey-bin.webp",
              "card": "videos/gallery/01_real/table1-put-all-the-red-things-in-the-grey-bin_card.mp4",
              "title": "Put red things in grey bin",
              "caption": "",
              "prompt": "Put all the red things in the grey bin"
            },
            {
              "file": "videos/gallery/01_real/table1-pack-canned-foods-into-the-bin.mp4",
              "thumb": "thumbnails/gallery/01_real/table1-pack-canned-foods-into-the-bin.webp",
              "card": "videos/gallery/01_real/table1-pack-canned-foods-into-the-bin_card.mp4",
              "title": "Pack canned foods into bin",
              "caption": "",
              "prompt": "Pack canned foods into the bin"
            },
            {
              "file": "videos/gallery/01_real/table1-put-all-the-lemons-and-the-orange-on-the-plate.mp4",
              "thumb": "thumbnails/gallery/01_real/table1-put-all-the-lemons-and-the-orange-on-the-plate.webp",
              "card": "videos/gallery/01_real/table1-put-all-the-lemons-and-the-orange-on-the-plate_card.mp4",
              "title": "Put lemons and orange on plate",
              "caption": "",
              "prompt": "Put all the lemons and the orange on the plate"
            },
            {
              "file": "videos/gallery/01_real/table1-put-all-the-tools-except-the-hammer-in-the-right-bin.mp4",
              "thumb": "thumbnails/gallery/01_real/table1-put-all-the-tools-except-the-hammer-in-the-right-bin.webp",
              "card": "videos/gallery/01_real/table1-put-all-the-tools-except-the-hammer-in-the-right-bin_card.mp4",
              "title": "Put tools except hammer in right bin",
              "caption": "",
              "prompt": "Put all the tools except the hammer in the right bin"
            },
            {
              "file": "videos/gallery/01_real/table1-pour-the-blue-cubes-onto-the-plate.mp4",
              "thumb": "thumbnails/gallery/01_real/table1-pour-the-blue-cubes-onto-the-plate.webp",
              "card": "videos/gallery/01_real/table1-pour-the-blue-cubes-onto-the-plate_card.mp4",
              "title": "Pour blue cubes onto plate",
              "caption": "",
              "prompt": "Pour the blue cubes onto the plate"
            },
            {
              "file": "videos/gallery/01_real/table1-pick-up-the-yellow-cube.mp4",
              "thumb": "thumbnails/gallery/01_real/table1-pick-up-the-yellow-cube.webp",
              "card": "videos/gallery/01_real/table1-pick-up-the-yellow-cube_card.mp4",
              "title": "Pick up yellow cube",
              "caption": "",
              "prompt": "Pick up the yellow cube"
            },
            {
              "file": "videos/gallery/01_real/table1-use-the-spatula-to-push-the-four-cubes-into-a-straight-aligned-row.mp4",
              "thumb": "thumbnails/gallery/01_real/table1-use-the-spatula-to-push-the-four-cubes-into-a-straight-aligned-row.webp",
              "card": "videos/gallery/01_real/table1-use-the-spatula-to-push-the-four-cubes-into-a-straight-aligned-row_card.mp4",
              "title": "Push cubes into a row with spatula",
              "caption": "",
              "prompt": "Use the spatula to push the four cubes into a straight, aligned row"
            },
            {
              "file": "videos/gallery/01_real/table1-cover-each-cube-with-a-cup-then-uncover-the-yellow-cube.mp4",
              "thumb": "thumbnails/gallery/01_real/table1-cover-each-cube-with-a-cup-then-uncover-the-yellow-cube.webp",
              "card": "videos/gallery/01_real/table1-cover-each-cube-with-a-cup-then-uncover-the-yellow-cube_card.mp4",
              "title": "Cover cubes with cups, uncover yellow",
              "caption": "",
              "prompt": "Cover each cube with a cup, then uncover the yellow cube"
            },
            {
              "file": "videos/gallery/01_real/table1-nest-the-cups-inside-the-largest-cup.mp4",
              "thumb": "thumbnails/gallery/01_real/table1-nest-the-cups-inside-the-largest-cup.webp",
              "card": "videos/gallery/01_real/table1-nest-the-cups-inside-the-largest-cup_card.mp4",
              "title": "Nest cups in largest cup",
              "caption": "",
              "prompt": "Nest the cups inside the largest cup"
            }
          ]
        },
        {
          "name": "Simulation (zero-shot)",
          "videos": [
            {
              "file": "videos/gallery/02_simulation/hook-mug-upright.mp4",
              "thumb": "thumbnails/gallery/02_simulation/hook-mug-upright.webp",
              "card": "videos/gallery/02_simulation/hook-mug-upright_card.mp4",
              "title": "Reorient mugs upright",
              "caption": "",
              "prompt": "Reorient all the mugs upright so that the opening is facing upwards.",
              "emergent": "Reorienting objects by hooking and pivoting the gripper",
              "benchmark": "RoboLab-120"
            },
            {
              "file": "videos/gallery/02_simulation/find-unseen-beam.mp4",
              "thumb": "thumbnails/gallery/02_simulation/find-unseen-beam.webp",
              "card": "videos/gallery/02_simulation/find-unseen-beam_card.mp4",
              "title": "Put jugs on shelf",
              "caption": "",
              "prompt": "Put all the jugs on the shelf",
              "emergent": "Inferring environmental constraints from physical interaction",
              "benchmark": "RoboLab-120"
            },
            {
              "file": "videos/gallery/02_simulation/cradle-drill.mp4",
              "thumb": "thumbnails/gallery/02_simulation/cradle-drill.webp",
              "card": "videos/gallery/02_simulation/cradle-drill_card.mp4",
              "title": "Put drill on table",
              "caption": "",
              "prompt": "Select the cordless drill and put it on the table",
              "emergent": "Mechanical interlocking grasps for heavy objects",
              "benchmark": "RoboLab-120"
            },
            {
              "file": "videos/gallery/02_simulation/robolab-mugs-on-shelf.mp4",
              "thumb": "thumbnails/gallery/02_simulation/robolab-mugs-on-shelf.webp",
              "card": "videos/gallery/02_simulation/robolab-mugs-on-shelf_card.mp4",
              "title": "Put mugs on shelf",
              "caption": "",
              "prompt": "Put the two mugs on the shelf",
              "benchmark": "RoboLab-120"
            },
            {
              "file": "videos/gallery/02_simulation/robolab-stack-blocks.mp4",
              "thumb": "thumbnails/gallery/02_simulation/robolab-stack-blocks.webp",
              "card": "videos/gallery/02_simulation/robolab-stack-blocks_card.mp4",
              "title": "Stack blocks in order",
              "caption": "",
              "prompt": "Stack the blocks in the order from bottom to top: red, blue, green, yellow",
              "benchmark": "RoboLab-120"
            },
            {
              "file": "videos/gallery/02_simulation/roll-pomegranate.mp4",
              "thumb": "thumbnails/gallery/02_simulation/roll-pomegranate.webp",
              "card": "videos/gallery/02_simulation/roll-pomegranate_card.mp4",
              "title": "Clear away organic objects",
              "caption": "",
              "prompt": "Clear away the organic objects",
              "emergent": "Scooping objects too wide to grasp",
              "benchmark": "RoboLab-120"
            },
            {
              "file": "videos/gallery/02_simulation/robolab-stand-jug-upright.mp4",
              "thumb": "thumbnails/gallery/02_simulation/robolab-stand-jug-upright.webp",
              "card": "videos/gallery/02_simulation/robolab-stand-jug-upright_card.mp4",
              "title": "Stand jug upright",
              "caption": "",
              "prompt": "Stand the jug upright",
              "benchmark": "RoboLab-120"
            },
            {
              "file": "videos/gallery/02_simulation/robolab-recycle-cartons.mp4",
              "thumb": "thumbnails/gallery/02_simulation/robolab-recycle-cartons.webp",
              "card": "videos/gallery/02_simulation/robolab-recycle-cartons_card.mp4",
              "title": "Recycle cartons",
              "caption": "",
              "prompt": "Put the cartons that can be recycled in the vertical crate",
              "benchmark": "RoboLab-120"
            },
            {
              "file": "videos/gallery/02_simulation/robolab-marker-in-mug.mp4",
              "thumb": "thumbnails/gallery/02_simulation/robolab-marker-in-mug.webp",
              "card": "videos/gallery/02_simulation/robolab-marker-in-mug_card.mp4",
              "title": "Put marker in mug",
              "caption": "",
              "prompt": "Put the whiteboard marker in the mug",
              "benchmark": "RoboLab-120"
            },
            {
              "file": "videos/gallery/02_simulation/robolab-clear-the-plate.mp4",
              "thumb": "thumbnails/gallery/02_simulation/robolab-clear-the-plate.webp",
              "card": "videos/gallery/02_simulation/robolab-clear-the-plate_card.mp4",
              "title": "Clear the plate",
              "caption": "",
              "prompt": "Put the two measuring cups outside of the plate",
              "benchmark": "RoboLab-120"
            },
            {
              "file": "videos/gallery/02_simulation/robolab-white-and-yellow-objects.mp4",
              "thumb": "thumbnails/gallery/02_simulation/robolab-white-and-yellow-objects.webp",
              "card": "videos/gallery/02_simulation/robolab-white-and-yellow-objects_card.mp4",
              "title": "Bin the white and yellow objects",
              "caption": "",
              "prompt": "Put all white objects and yellow objects in the grey bin",
              "benchmark": "RoboLab-120"
            },
            {
              "file": "videos/gallery/02_simulation/robolab-larger-raisin-box.mp4",
              "thumb": "thumbnails/gallery/02_simulation/robolab-larger-raisin-box.webp",
              "card": "videos/gallery/02_simulation/robolab-larger-raisin-box_card.mp4",
              "title": "Bin the larger raisin box",
              "caption": "",
              "prompt": "Place the larger object in the grey bin.",
              "benchmark": "RoboLab-120"
            },
            {
              "file": "videos/gallery/02_simulation/robolab-bagels-on-plate.mp4",
              "thumb": "thumbnails/gallery/02_simulation/robolab-bagels-on-plate.webp",
              "card": "videos/gallery/02_simulation/robolab-bagels-on-plate_card.mp4",
              "title": "Put the bagels on the plate",
              "caption": "",
              "prompt": "Put the bagels on the plate",
              "benchmark": "RoboLab-120"
            },
            {
              "file": "videos/gallery/02_simulation/robolab-three-bananas-in-bin.mp4",
              "thumb": "thumbnails/gallery/02_simulation/robolab-three-bananas-in-bin.webp",
              "card": "videos/gallery/02_simulation/robolab-three-bananas-in-bin_card.mp4",
              "title": "Make it three bananas in the bin",
              "caption": "",
              "prompt": "Make sure there are 3 (three) bananas in the grey bin.",
              "benchmark": "RoboLab-120"
            },
            {
              "file": "videos/gallery/02_simulation/robolab-hammers-in-right-bin.mp4",
              "thumb": "thumbnails/gallery/02_simulation/robolab-hammers-in-right-bin.webp",
              "card": "videos/gallery/02_simulation/robolab-hammers-in-right-bin_card.mp4",
              "title": "Put the hammers in the right bin",
              "caption": "",
              "prompt": "Put hammers in the right bin and do not touch anything else",
              "benchmark": "RoboLab-120"
            },
            {
              "file": "videos/gallery/02_simulation/robolab-plastic-bottles-in-bin.mp4",
              "thumb": "thumbnails/gallery/02_simulation/robolab-plastic-bottles-in-bin.webp",
              "card": "videos/gallery/02_simulation/robolab-plastic-bottles-in-bin_card.mp4",
              "title": "Put the plastic bottles away",
              "caption": "",
              "prompt": "Put all plastic bottles away in the bin",
              "benchmark": "RoboLab-120"
            },
            {
              "file": "videos/gallery/02_simulation/robolab-canned-food-in-bin.mp4",
              "thumb": "thumbnails/gallery/02_simulation/robolab-canned-food-in-bin.webp",
              "card": "videos/gallery/02_simulation/robolab-canned-food-in-bin_card.mp4",
              "title": "Put the canned food in the bin",
              "caption": "",
              "prompt": "Put the canned food in the grey bin",
              "benchmark": "RoboLab-120"
            },
            {
              "file": "videos/gallery/02_simulation/robolab-coffee-pot-in-bin.mp4",
              "thumb": "thumbnails/gallery/02_simulation/robolab-coffee-pot-in-bin.webp",
              "card": "videos/gallery/02_simulation/robolab-coffee-pot-in-bin_card.mp4",
              "title": "Put the coffee pot in the bin",
              "caption": "",
              "prompt": "Put the coffee pot in the grey bin",
              "benchmark": "RoboLab-120"
            },
            {
              "file": "videos/gallery/02_simulation/robolab-pack-boxed-food.mp4",
              "thumb": "thumbnails/gallery/02_simulation/robolab-pack-boxed-food.webp",
              "card": "videos/gallery/02_simulation/robolab-pack-boxed-food_card.mp4",
              "title": "Pack the boxed food",
              "caption": "",
              "prompt": "Pack boxed foods into the bin",
              "benchmark": "RoboLab-120"
            },
            {
              "file": "videos/gallery/02_simulation/robolab-three-fruits-on-plate.mp4",
              "thumb": "thumbnails/gallery/02_simulation/robolab-three-fruits-on-plate.webp",
              "card": "videos/gallery/02_simulation/robolab-three-fruits-on-plate_card.mp4",
              "title": "Put three fruits on the plate",
              "caption": "",
              "prompt": "Put three (3) fruits on the plate",
              "benchmark": "RoboLab-120"
            },
            {
              "file": "videos/gallery/02_simulation/robolab-spatula-off-shelf.mp4",
              "thumb": "thumbnails/gallery/02_simulation/robolab-spatula-off-shelf.webp",
              "card": "videos/gallery/02_simulation/robolab-spatula-off-shelf_card.mp4",
              "title": "Take the spatula off the shelf",
              "caption": "",
              "prompt": "Take the spatula off the shelf and put it on the table",
              "benchmark": "RoboLab-120"
            },
            {
              "file": "videos/gallery/02_simulation/robolab-mustard-in-right-bin.mp4",
              "thumb": "thumbnails/gallery/02_simulation/robolab-mustard-in-right-bin.webp",
              "card": "videos/gallery/02_simulation/robolab-mustard-in-right-bin_card.mp4",
              "title": "Put the mustard in the right bin",
              "caption": "",
              "prompt": "Put the mustard in the right bin",
              "benchmark": "RoboLab-120"
            },
            {
              "file": "videos/gallery/02_simulation/robolab-pick-up-blue-pitcher.mp4",
              "thumb": "thumbnails/gallery/02_simulation/robolab-pick-up-blue-pitcher.webp",
              "card": "videos/gallery/02_simulation/robolab-pick-up-blue-pitcher_card.mp4",
              "title": "Pick up the blue pitcher",
              "caption": "",
              "prompt": "Pick up the large blue pitcher",
              "benchmark": "RoboLab-120"
            },
            {
              "file": "videos/gallery/02_simulation/robolab-pink-spoon-in-pot.mp4",
              "thumb": "thumbnails/gallery/02_simulation/robolab-pink-spoon-in-pot.webp",
              "card": "videos/gallery/02_simulation/robolab-pink-spoon-in-pot_card.mp4",
              "title": "Put the pink spoon in the pot",
              "caption": "",
              "prompt": "Put the pink spaghetti spoon in the pot",
              "benchmark": "RoboLab-120"
            },
            {
              "file": "videos/gallery/02_simulation/robolab-bowl-on-shelf.mp4",
              "thumb": "thumbnails/gallery/02_simulation/robolab-bowl-on-shelf.webp",
              "card": "videos/gallery/02_simulation/robolab-bowl-on-shelf_card.mp4",
              "title": "Put the bowl on the shelf",
              "caption": "",
              "prompt": "Put the serving bowl anywhere on the shelf in front of you",
              "benchmark": "RoboLab-120"
            },
            {
              "file": "videos/gallery/02_simulation/robolab-stack-yellow-on-red.mp4",
              "thumb": "thumbnails/gallery/02_simulation/robolab-stack-yellow-on-red.webp",
              "card": "videos/gallery/02_simulation/robolab-stack-yellow-on-red_card.mp4",
              "title": "Stack yellow on red",
              "caption": "",
              "prompt": "Stack the yellow block on the red block",
              "benchmark": "RoboLab-120"
            },
            {
              "file": "videos/gallery/02_simulation/robolab-wooden-spatula-in-bowl.mp4",
              "thumb": "thumbnails/gallery/02_simulation/robolab-wooden-spatula-in-bowl.webp",
              "card": "videos/gallery/02_simulation/robolab-wooden-spatula-in-bowl_card.mp4",
              "title": "Put the wooden spatula in the bowl",
              "caption": "",
              "prompt": "Put the wooden spatula in the bowl",
              "benchmark": "RoboLab-120"
            },
            {
              "file": "videos/gallery/02_simulation/robolab-yogurt-in-bowl.mp4",
              "thumb": "thumbnails/gallery/02_simulation/robolab-yogurt-in-bowl.webp",
              "card": "videos/gallery/02_simulation/robolab-yogurt-in-bowl_card.mp4",
              "title": "Put the yogurt in the bowl",
              "caption": "",
              "prompt": "Put the small red yogurt in the red bowl",
              "benchmark": "RoboLab-120"
            }
          ]
        },
        {
          "name": "Simulation (few-shot)",
          "videos": [
            {
              "file": "videos/gallery/03_few-shot/robodojo-fold-clothes.mp4",
              "thumb": "thumbnails/gallery/03_few-shot/robodojo-fold-clothes.webp",
              "card": "videos/gallery/03_few-shot/robodojo-fold-clothes_card.mp4",
              "title": "Fold clothes",
              "caption": "",
              "prompt": "Fold the clothes neatly.",
              "benchmark": "RoboDojo"
            },
            {
              "file": "videos/gallery/03_few-shot/robodojo-organize-table.mp4",
              "thumb": "thumbnails/gallery/03_few-shot/robodojo-organize-table.webp",
              "card": "videos/gallery/03_few-shot/robodojo-organize-table_card.mp4",
              "title": "Organize a table",
              "caption": "",
              "prompt": "Place the alarm clock on the drawer, put the figurine on the stand, place the mouse on the mouse pad, and push the keyboard into the frame.",
              "benchmark": "RoboDojo"
            },
            {
              "file": "videos/gallery/03_few-shot/robodojo-classify-objects.mp4",
              "thumb": "thumbnails/gallery/03_few-shot/robodojo-classify-objects.webp",
              "card": "videos/gallery/03_few-shot/robodojo-classify-objects_card.mp4",
              "title": "Classify objects",
              "caption": "",
              "prompt": "Sort the objects by category into the three baskets.",
              "benchmark": "RoboDojo"
            },
            {
              "file": "videos/gallery/03_few-shot/robodojo-pour-by-language.mp4",
              "thumb": "thumbnails/gallery/03_few-shot/robodojo-pour-by-language.webp",
              "card": "videos/gallery/03_few-shot/robodojo-pour-by-language_card.mp4",
              "title": "Pour by language",
              "caption": "",
              "prompt": "Pour the liquid from the first turquoise bottle into the first black bowl, from the second violet bottle into the second white bowl, and from the third red bottle into the third brown bowl. Then reset the robot arm.",
              "benchmark": "RoboDojo"
            },
            {
              "file": "videos/gallery/03_few-shot/robodojo-pick-by-image.mp4",
              "thumb": "thumbnails/gallery/03_few-shot/robodojo-pick-by-image.webp",
              "card": "videos/gallery/03_few-shot/robodojo-pick-by-image_card.mp4",
              "title": "Pick from a conveyor by image",
              "caption": "",
              "prompt": "Lift the basket more than 8 cm, identify the target object on the conveyor according to the image on the board, pick it up, and place it into the basket.",
              "benchmark": "RoboDojo"
            },
            {
              "file": "videos/gallery/03_few-shot/robodojo-stack-bowls.mp4",
              "thumb": "thumbnails/gallery/03_few-shot/robodojo-stack-bowls.webp",
              "card": "videos/gallery/03_few-shot/robodojo-stack-bowls_card.mp4",
              "title": "Stack bowls",
              "caption": "",
              "prompt": "Stack the three bowls together.",
              "benchmark": "RoboDojo"
            },
            {
              "file": "videos/gallery/03_few-shot/robodojo-make-toast.mp4",
              "thumb": "thumbnails/gallery/03_few-shot/robodojo-make-toast.webp",
              "card": "videos/gallery/03_few-shot/robodojo-make-toast_card.mp4",
              "title": "Make toast",
              "caption": "",
              "prompt": "Pick up two slices of bread, place them into the toaster, and press the lever down.",
              "benchmark": "RoboDojo"
            },
            {
              "file": "videos/gallery/03_few-shot/robodojo-declare-kong.mp4",
              "thumb": "thumbnails/gallery/03_few-shot/robodojo-declare-kong.webp",
              "card": "videos/gallery/03_few-shot/robodojo-declare-kong_card.mp4",
              "title": "Declare a kong",
              "caption": "",
              "prompt": "Wait for the opponent to discard a tile, then declare a kong with the matching tiles.",
              "benchmark": "RoboDojo"
            },
            {
              "file": "videos/gallery/03_few-shot/robodojo-store-laptop.mp4",
              "thumb": "thumbnails/gallery/03_few-shot/robodojo-store-laptop.webp",
              "card": "videos/gallery/03_few-shot/robodojo-store-laptop_card.mp4",
              "title": "Store a laptop and headphones",
              "caption": "",
              "prompt": "Hang the headphones on the headphone stand, close the laptop, then place it into the vertical laptop stand.",
              "benchmark": "RoboDojo"
            },
            {
              "file": "videos/gallery/03_few-shot/robodojo-build-tower.mp4",
              "thumb": "thumbnails/gallery/03_few-shot/robodojo-build-tower.webp",
              "card": "videos/gallery/03_few-shot/robodojo-build-tower_card.mp4",
              "title": "Build a tower",
              "caption": "",
              "prompt": "Build a tower using the wooden blocks and wooden boards.",
              "benchmark": "RoboDojo"
            },
            {
              "file": "videos/gallery/03_few-shot/robodojo-hang-mugs.mp4",
              "thumb": "thumbnails/gallery/03_few-shot/robodojo-hang-mugs.webp",
              "card": "videos/gallery/03_few-shot/robodojo-hang-mugs_card.mp4",
              "title": "Hang mugs",
              "caption": "",
              "prompt": "Hang all the mugs on the mug rack.",
              "benchmark": "RoboDojo"
            },
            {
              "file": "videos/gallery/03_few-shot/robodojo-insert-tubes.mp4",
              "thumb": "thumbnails/gallery/03_few-shot/robodojo-insert-tubes.webp",
              "card": "videos/gallery/03_few-shot/robodojo-insert-tubes_card.mp4",
              "title": "Insert tubes",
              "caption": "",
              "prompt": "Insert the three tubes into the rack one by one.",
              "benchmark": "RoboDojo"
            },
            {
              "file": "videos/gallery/03_few-shot/robodojo-fill-pen-holder.mp4",
              "thumb": "thumbnails/gallery/03_few-shot/robodojo-fill-pen-holder.webp",
              "card": "videos/gallery/03_few-shot/robodojo-fill-pen-holder_card.mp4",
              "title": "Fill a pen holder",
              "caption": "",
              "prompt": "Hold the pen holder with one hand, place all pens into it with the other hand, then put it back down.",
              "benchmark": "RoboDojo"
            },
            {
              "file": "videos/gallery/03_few-shot/robodojo-tic-tac-toe.mp4",
              "thumb": "thumbnails/gallery/03_few-shot/robodojo-tic-tac-toe.webp",
              "card": "videos/gallery/03_few-shot/robodojo-tic-tac-toe_card.mp4",
              "title": "Play tic-tac-toe",
              "caption": "",
              "prompt": "Play tic-tac-toe as the first player and fill the board with the opponent.",
              "benchmark": "RoboDojo"
            },
            {
              "file": "videos/gallery/03_few-shot/robodojo-stacking-toy.mp4",
              "thumb": "thumbnails/gallery/03_few-shot/robodojo-stacking-toy.webp",
              "card": "videos/gallery/03_few-shot/robodojo-stacking-toy_card.mp4",
              "title": "Play with a stacking toy",
              "caption": "",
              "prompt": "Place all stacking toy pieces onto the correct pegs.",
              "benchmark": "RoboDojo"
            },
            {
              "file": "videos/gallery/03_few-shot/robodojo-bottles-in-dustbin.mp4",
              "thumb": "thumbnails/gallery/03_few-shot/robodojo-bottles-in-dustbin.webp",
              "card": "videos/gallery/03_few-shot/robodojo-bottles-in-dustbin_card.mp4",
              "title": "Put bottles into a dustbin",
              "caption": "",
              "prompt": "Pick up the bottles and throw them into the dustbin, using handover when needed.",
              "benchmark": "RoboDojo"
            }
          ]
        }
      ]
    },
    "realworld": {
      "categories": [
        {
          "name": "AMP1",
          "videos": [
            {
              "file": "videos/realworld/01_AMP1/01-put-the-rubiks-cube-to-the-left-of-the-bowl.mp4",
              "thumb": "thumbnails/realworld/01_AMP1/01-put-the-rubiks-cube-to-the-left-of-the-bowl.webp",
              "title": "Put the rubiks cube to the left of the bowl",
              "caption": "",
              "prompt": "Put the rubiks cube to the left of the bowl",
              "note": "Succeeded"
            },
            {
              "file": "videos/realworld/01_AMP1/02-put-the-cube-and-the-banana-in-the-bowl.mp4",
              "thumb": "thumbnails/realworld/01_AMP1/02-put-the-cube-and-the-banana-in-the-bowl.webp",
              "title": "Put the cube and the banana in the bowl",
              "caption": "",
              "prompt": "Put the cube and the banana in the bowl",
              "note": "Succeeded"
            },
            {
              "file": "videos/realworld/01_AMP1/03-put-all-the-red-things-in-the-grey-bin.mp4",
              "thumb": "thumbnails/realworld/01_AMP1/03-put-all-the-red-things-in-the-grey-bin.webp",
              "title": "Put all the red things in the grey bin",
              "caption": "",
              "prompt": "Put all the red things in the grey bin",
              "note": "Succeeded"
            },
            {
              "file": "videos/realworld/01_AMP1/04-pack-canned-foods-into-the-bin.mp4",
              "thumb": "thumbnails/realworld/01_AMP1/04-pack-canned-foods-into-the-bin.webp",
              "title": "Pack canned foods into the bin",
              "caption": "",
              "prompt": "Pack canned foods into the bin",
              "note": "Succeeded"
            },
            {
              "file": "videos/realworld/01_AMP1/05-put-all-the-lemons-and-the-orange-on-the-plate.mp4",
              "thumb": "thumbnails/realworld/01_AMP1/05-put-all-the-lemons-and-the-orange-on-the-plate.webp",
              "title": "Put all the lemons and the orange on the plate",
              "caption": "",
              "prompt": "Put all the lemons and the orange on the plate",
              "note": "Succeeded"
            },
            {
              "file": "videos/realworld/01_AMP1/06-put-all-the-tools-except-the-hammer-in-the-right-bin.mp4",
              "thumb": "thumbnails/realworld/01_AMP1/06-put-all-the-tools-except-the-hammer-in-the-right-bin.webp",
              "title": "Put all the tools except the hammer in the right bin",
              "caption": "",
              "prompt": "Put all the tools except the hammer in the right bin",
              "note": "Succeeded"
            },
            {
              "file": "videos/realworld/01_AMP1/07-pour-the-blue-cubes-onto-the-plate.mp4",
              "thumb": "thumbnails/realworld/01_AMP1/07-pour-the-blue-cubes-onto-the-plate.webp",
              "title": "Pour the blue cubes onto the plate",
              "caption": "",
              "prompt": "Pour the blue cubes onto the plate",
              "note": "Succeeded"
            },
            {
              "file": "videos/realworld/01_AMP1/08-put-the-bowl-on-the-plate.mp4",
              "thumb": "thumbnails/realworld/01_AMP1/08-put-the-bowl-on-the-plate.webp",
              "title": "Put the bowl on the plate",
              "caption": "",
              "prompt": "Put the bowl on the plate",
              "note": "Succeeded"
            },
            {
              "file": "videos/realworld/01_AMP1/09-nest-the-cups-inside-the-largest-cup.mp4",
              "thumb": "thumbnails/realworld/01_AMP1/09-nest-the-cups-inside-the-largest-cup.webp",
              "title": "Nest the cups inside the largest cup",
              "caption": "",
              "prompt": "Nest the cups inside the largest cup",
              "note": "Succeeded"
            },
            {
              "file": "videos/realworld/01_AMP1/10-pick-up-the-yellow-cube.mp4",
              "thumb": "thumbnails/realworld/01_AMP1/10-pick-up-the-yellow-cube.webp",
              "title": "Pick up the yellow cube",
              "caption": "",
              "prompt": "Pick up the yellow cube",
              "note": "Succeeded"
            },
            {
              "file": "videos/realworld/01_AMP1/11-use-the-spatula-to-push-the-four-cubes-into-a-straight-aligned-row.mp4",
              "thumb": "thumbnails/realworld/01_AMP1/11-use-the-spatula-to-push-the-four-cubes-into-a-straight-aligned-row.webp",
              "title": "Use the spatula to push the four cubes into a straight, aligned row",
              "caption": "",
              "prompt": "Use the spatula to push the four cubes into a straight, aligned row",
              "note": "Succeeded"
            },
            {
              "file": "videos/realworld/01_AMP1/12-cover-each-cube-with-a-cup-then-uncover-the-yellow-cube.mp4",
              "thumb": "thumbnails/realworld/01_AMP1/12-cover-each-cube-with-a-cup-then-uncover-the-yellow-cube.webp",
              "title": "Cover each cube with a cup, then uncover the yellow cube",
              "caption": "",
              "prompt": "Cover each cube with a cup, then uncover the yellow cube",
              "note": "Succeeded"
            }
          ]
        },
        {
          "name": "FLUX 3 Action",
          "videos": [
            {
              "file": "videos/realworld/02_FLUX_3_Action/01-put-the-rubiks-cube-to-the-left-of-the-bowl.mp4",
              "thumb": "thumbnails/realworld/02_FLUX_3_Action/01-put-the-rubiks-cube-to-the-left-of-the-bowl.webp",
              "title": "Put the rubiks cube to the left of the bowl",
              "caption": "",
              "prompt": "Put the rubiks cube to the left of the bowl",
              "note": "Succeeded"
            },
            {
              "file": "videos/realworld/02_FLUX_3_Action/02-put-the-cube-and-the-banana-in-the-bowl.mp4",
              "thumb": "thumbnails/realworld/02_FLUX_3_Action/02-put-the-cube-and-the-banana-in-the-bowl.webp",
              "title": "Put the cube and the banana in the bowl",
              "caption": "",
              "prompt": "Put the cube and the banana in the bowl",
              "note": "Succeeded"
            },
            {
              "file": "videos/realworld/02_FLUX_3_Action/03-put-all-the-red-things-in-the-grey-bin.mp4",
              "thumb": "thumbnails/realworld/02_FLUX_3_Action/03-put-all-the-red-things-in-the-grey-bin.webp",
              "title": "Put all the red things in the grey bin",
              "caption": "",
              "prompt": "Put all the red things in the grey bin",
              "note": "Failed"
            },
            {
              "file": "videos/realworld/02_FLUX_3_Action/04-pack-canned-foods-into-the-bin.mp4",
              "thumb": "thumbnails/realworld/02_FLUX_3_Action/04-pack-canned-foods-into-the-bin.webp",
              "title": "Pack canned foods into the bin",
              "caption": "",
              "prompt": "Pack canned foods into the bin",
              "note": "Succeeded"
            },
            {
              "file": "videos/realworld/02_FLUX_3_Action/05-put-all-the-lemons-and-the-orange-on-the-plate.mp4",
              "thumb": "thumbnails/realworld/02_FLUX_3_Action/05-put-all-the-lemons-and-the-orange-on-the-plate.webp",
              "title": "Put all the lemons and the orange on the plate",
              "caption": "",
              "prompt": "Put all the lemons and the orange on the plate",
              "note": "Failed"
            },
            {
              "file": "videos/realworld/02_FLUX_3_Action/06-put-all-the-tools-except-the-hammer-in-the-right-bin.mp4",
              "thumb": "thumbnails/realworld/02_FLUX_3_Action/06-put-all-the-tools-except-the-hammer-in-the-right-bin.webp",
              "title": "Put all the tools except the hammer in the right bin",
              "caption": "",
              "prompt": "Put all the tools except the hammer in the right bin",
              "note": "Succeeded"
            },
            {
              "file": "videos/realworld/02_FLUX_3_Action/07-pour-the-blue-cubes-onto-the-plate.mp4",
              "thumb": "thumbnails/realworld/02_FLUX_3_Action/07-pour-the-blue-cubes-onto-the-plate.webp",
              "title": "Pour the blue cubes onto the plate",
              "caption": "",
              "prompt": "Pour the blue cubes onto the plate",
              "note": "Succeeded"
            },
            {
              "file": "videos/realworld/02_FLUX_3_Action/08-put-the-bowl-on-the-plate.mp4",
              "thumb": "thumbnails/realworld/02_FLUX_3_Action/08-put-the-bowl-on-the-plate.webp",
              "title": "Put the bowl on the plate",
              "caption": "",
              "prompt": "Put the bowl on the plate",
              "note": "Failed"
            },
            {
              "file": "videos/realworld/02_FLUX_3_Action/09-nest-the-cups-inside-the-largest-cup.mp4",
              "thumb": "thumbnails/realworld/02_FLUX_3_Action/09-nest-the-cups-inside-the-largest-cup.webp",
              "title": "Nest the cups inside the largest cup",
              "caption": "",
              "prompt": "Nest the cups inside the largest cup",
              "note": "Failed"
            },
            {
              "file": "videos/realworld/02_FLUX_3_Action/10-pick-up-the-yellow-cube.mp4",
              "thumb": "thumbnails/realworld/02_FLUX_3_Action/10-pick-up-the-yellow-cube.webp",
              "title": "Pick up the yellow cube",
              "caption": "",
              "prompt": "Pick up the yellow cube",
              "note": "Succeeded"
            },
            {
              "file": "videos/realworld/02_FLUX_3_Action/11-use-the-spatula-to-push-the-four-cubes-into-a-straight-aligned-row.mp4",
              "thumb": "thumbnails/realworld/02_FLUX_3_Action/11-use-the-spatula-to-push-the-four-cubes-into-a-straight-aligned-row.webp",
              "title": "Use the spatula to push the four cubes into a straight, aligned row",
              "caption": "",
              "prompt": "Use the spatula to push the four cubes into a straight, aligned row",
              "note": "Failed"
            },
            {
              "file": "videos/realworld/02_FLUX_3_Action/12-cover-each-cube-with-a-cup-then-uncover-the-yellow-cube.mp4",
              "thumb": "thumbnails/realworld/02_FLUX_3_Action/12-cover-each-cube-with-a-cup-then-uncover-the-yellow-cube.webp",
              "title": "Cover each cube with a cup, then uncover the yellow cube",
              "caption": "",
              "prompt": "Cover each cube with a cup, then uncover the yellow cube",
              "note": "Failed"
            }
          ]
        },
        {
          "name": "π0.5",
          "videos": [
            {
              "file": "videos/realworld/03_pi0.5/01-put-the-rubiks-cube-to-the-left-of-the-bowl.mp4",
              "thumb": "thumbnails/realworld/03_pi0.5/01-put-the-rubiks-cube-to-the-left-of-the-bowl.webp",
              "title": "Put the rubiks cube to the left of the bowl",
              "caption": "",
              "prompt": "Put the rubiks cube to the left of the bowl",
              "note": "Failed"
            },
            {
              "file": "videos/realworld/03_pi0.5/02-put-the-cube-and-the-banana-in-the-bowl.mp4",
              "thumb": "thumbnails/realworld/03_pi0.5/02-put-the-cube-and-the-banana-in-the-bowl.webp",
              "title": "Put the cube and the banana in the bowl",
              "caption": "",
              "prompt": "Put the cube and the banana in the bowl",
              "note": "Succeeded"
            },
            {
              "file": "videos/realworld/03_pi0.5/03-put-all-the-red-things-in-the-grey-bin.mp4",
              "thumb": "thumbnails/realworld/03_pi0.5/03-put-all-the-red-things-in-the-grey-bin.webp",
              "title": "Put all the red things in the grey bin",
              "caption": "",
              "prompt": "Put all the red things in the grey bin",
              "note": "Failed"
            },
            {
              "file": "videos/realworld/03_pi0.5/04-pack-canned-foods-into-the-bin.mp4",
              "thumb": "thumbnails/realworld/03_pi0.5/04-pack-canned-foods-into-the-bin.webp",
              "title": "Pack canned foods into the bin",
              "caption": "",
              "prompt": "Pack canned foods into the bin",
              "note": "Failed"
            },
            {
              "file": "videos/realworld/03_pi0.5/05-put-all-the-lemons-and-the-orange-on-the-plate.mp4",
              "thumb": "thumbnails/realworld/03_pi0.5/05-put-all-the-lemons-and-the-orange-on-the-plate.webp",
              "title": "Put all the lemons and the orange on the plate",
              "caption": "",
              "prompt": "Put all the lemons and the orange on the plate",
              "note": "Failed"
            },
            {
              "file": "videos/realworld/03_pi0.5/06-put-all-the-tools-except-the-hammer-in-the-right-bin.mp4",
              "thumb": "thumbnails/realworld/03_pi0.5/06-put-all-the-tools-except-the-hammer-in-the-right-bin.webp",
              "title": "Put all the tools except the hammer in the right bin",
              "caption": "",
              "prompt": "Put all the tools except the hammer in the right bin",
              "note": "Failed"
            },
            {
              "file": "videos/realworld/03_pi0.5/07-pour-the-blue-cubes-onto-the-plate.mp4",
              "thumb": "thumbnails/realworld/03_pi0.5/07-pour-the-blue-cubes-onto-the-plate.webp",
              "title": "Pour the blue cubes onto the plate",
              "caption": "",
              "prompt": "Pour the blue cubes onto the plate",
              "note": "Failed"
            },
            {
              "file": "videos/realworld/03_pi0.5/08-put-the-bowl-on-the-plate.mp4",
              "thumb": "thumbnails/realworld/03_pi0.5/08-put-the-bowl-on-the-plate.webp",
              "title": "Put the bowl on the plate",
              "caption": "",
              "prompt": "Put the bowl on the plate",
              "note": "Succeeded"
            },
            {
              "file": "videos/realworld/03_pi0.5/09-nest-the-cups-inside-the-largest-cup.mp4",
              "thumb": "thumbnails/realworld/03_pi0.5/09-nest-the-cups-inside-the-largest-cup.webp",
              "title": "Nest the cups inside the largest cup",
              "caption": "",
              "prompt": "Nest the cups inside the largest cup",
              "note": "Failed"
            },
            {
              "file": "videos/realworld/03_pi0.5/10-pick-up-the-yellow-cube.mp4",
              "thumb": "thumbnails/realworld/03_pi0.5/10-pick-up-the-yellow-cube.webp",
              "title": "Pick up the yellow cube",
              "caption": "",
              "prompt": "Pick up the yellow cube",
              "note": "Failed"
            },
            {
              "file": "videos/realworld/03_pi0.5/11-use-the-spatula-to-push-the-four-cubes-into-a-straight-aligned-row.mp4",
              "thumb": "thumbnails/realworld/03_pi0.5/11-use-the-spatula-to-push-the-four-cubes-into-a-straight-aligned-row.webp",
              "title": "Use the spatula to push the four cubes into a straight, aligned row",
              "caption": "",
              "prompt": "Use the spatula to push the four cubes into a straight, aligned row",
              "note": "Failed"
            },
            {
              "file": "videos/realworld/03_pi0.5/12-cover-each-cube-with-a-cup-then-uncover-the-yellow-cube.mp4",
              "thumb": "thumbnails/realworld/03_pi0.5/12-cover-each-cube-with-a-cup-then-uncover-the-yellow-cube.webp",
              "title": "Cover each cube with a cup, then uncover the yellow cube",
              "caption": "",
              "prompt": "Cover each cube with a cup, then uncover the yellow cube",
              "note": "Failed"
            }
          ]
        }
      ]
    },
    "robolab_default": {
      "categories": [
        {
          "name": "AMP1",
          "videos": [
            {
              "file": "videos/robolab_default/01_AMP1/12_stack-blocks.mp4",
              "thumb": "thumbnails/robolab_default/01_AMP1/12_stack-blocks.webp",
              "title": "Stack Blocks in Order",
              "caption": "",
              "prompt": "Stack the blocks in the order from bottom to top: red, blue, green, yellow",
              "note": "Succeeded"
            },
            {
              "file": "videos/robolab_default/01_AMP1/13_stand-jug-upright.mp4",
              "thumb": "thumbnails/robolab_default/01_AMP1/13_stand-jug-upright.webp",
              "title": "Stand the Jug Upright",
              "caption": "",
              "prompt": "Stand the jug upright",
              "note": "Succeeded"
            },
            {
              "file": "videos/robolab_default/01_AMP1/14_recycle-cartons.mp4",
              "thumb": "thumbnails/robolab_default/01_AMP1/14_recycle-cartons.webp",
              "title": "Recycle Cartons",
              "caption": "",
              "prompt": "Put the cartons that can be recycled in the vertical crate",
              "note": "Succeeded"
            },
            {
              "file": "videos/robolab_default/01_AMP1/15_jugs-on-shelf.mp4",
              "thumb": "thumbnails/robolab_default/01_AMP1/15_jugs-on-shelf.webp",
              "title": "Put Jugs on the Shelf",
              "caption": "",
              "prompt": "Put all the jugs on the shelf",
              "note": "Succeeded"
            },
            {
              "file": "videos/robolab_default/01_AMP1/03_white-and-yellow-objects.mp4",
              "thumb": "thumbnails/robolab_default/01_AMP1/03_white-and-yellow-objects.webp",
              "title": "Bin the White and Yellow Objects",
              "caption": "",
              "prompt": "Put all white objects and yellow objects in the grey bin",
              "note": "Succeeded"
            },
            {
              "file": "videos/robolab_default/01_AMP1/04_mustard-in-left-bin.mp4",
              "thumb": "thumbnails/robolab_default/01_AMP1/04_mustard-in-left-bin.webp",
              "title": "Put the Mustard in the Left Bin",
              "caption": "",
              "prompt": "Put the mustard in the left bin",
              "note": "Succeeded"
            },
            {
              "file": "videos/robolab_default/01_AMP1/05_larger-raisin-box.mp4",
              "thumb": "thumbnails/robolab_default/01_AMP1/05_larger-raisin-box.webp",
              "title": "Bin the Larger Raisin Box",
              "caption": "",
              "prompt": "Place the larger object in the grey bin.",
              "note": "Succeeded"
            },
            {
              "file": "videos/robolab_default/01_AMP1/07_marker-in-mug.mp4",
              "thumb": "thumbnails/robolab_default/01_AMP1/07_marker-in-mug.webp",
              "title": "Put the Marker in the Mug",
              "caption": "",
              "prompt": "Put the whiteboard marker in the mug",
              "note": "Succeeded"
            },
            {
              "file": "videos/robolab_default/01_AMP1/08_white-mugs-upright.mp4",
              "thumb": "thumbnails/robolab_default/01_AMP1/08_white-mugs-upright.webp",
              "title": "Stand the White Mugs Upright",
              "caption": "",
              "prompt": "Make sure all the white mugs are upright so that the opening is facing upwards.",
              "note": "Succeeded"
            },
            {
              "file": "videos/robolab_default/01_AMP1/10_clear-the-plate.mp4",
              "thumb": "thumbnails/robolab_default/01_AMP1/10_clear-the-plate.webp",
              "title": "Clear the Plate",
              "caption": "",
              "prompt": "Put the two measuring cups outside of the plate",
              "note": "Succeeded"
            }
          ]
        },
        {
          "name": "FLUX 3 Action",
          "videos": [
            {
              "file": "videos/robolab_default/02_FLUX_3_Action/12_stack-blocks.mp4",
              "thumb": "thumbnails/robolab_default/02_FLUX_3_Action/12_stack-blocks.webp",
              "title": "Stack Blocks in Order",
              "caption": "",
              "prompt": "Stack the blocks in the order from bottom to top: red, blue, green, yellow",
              "note": "Failed"
            },
            {
              "file": "videos/robolab_default/02_FLUX_3_Action/13_stand-jug-upright.mp4",
              "thumb": "thumbnails/robolab_default/02_FLUX_3_Action/13_stand-jug-upright.webp",
              "title": "Stand the Jug Upright",
              "caption": "",
              "prompt": "Stand the jug upright",
              "note": "Failed"
            },
            {
              "file": "videos/robolab_default/02_FLUX_3_Action/14_recycle-cartons.mp4",
              "thumb": "thumbnails/robolab_default/02_FLUX_3_Action/14_recycle-cartons.webp",
              "title": "Recycle Cartons",
              "caption": "",
              "prompt": "Put the cartons that can be recycled in the vertical crate",
              "note": "Failed"
            },
            {
              "file": "videos/robolab_default/02_FLUX_3_Action/15_jugs-on-shelf.mp4",
              "thumb": "thumbnails/robolab_default/02_FLUX_3_Action/15_jugs-on-shelf.webp",
              "title": "Put Jugs on the Shelf",
              "caption": "",
              "prompt": "Put all the jugs on the shelf",
              "note": "Failed"
            },
            {
              "file": "videos/robolab_default/02_FLUX_3_Action/03_white-and-yellow-objects.mp4",
              "thumb": "thumbnails/robolab_default/02_FLUX_3_Action/03_white-and-yellow-objects.webp",
              "title": "Bin the White and Yellow Objects",
              "caption": "",
              "prompt": "Put all white objects and yellow objects in the grey bin",
              "note": "Succeeded"
            },
            {
              "file": "videos/robolab_default/02_FLUX_3_Action/04_mustard-in-left-bin.mp4",
              "thumb": "thumbnails/robolab_default/02_FLUX_3_Action/04_mustard-in-left-bin.webp",
              "title": "Put the Mustard in the Left Bin",
              "caption": "",
              "prompt": "Put the mustard in the left bin",
              "note": "Succeeded"
            },
            {
              "file": "videos/robolab_default/02_FLUX_3_Action/05_larger-raisin-box.mp4",
              "thumb": "thumbnails/robolab_default/02_FLUX_3_Action/05_larger-raisin-box.webp",
              "title": "Bin the Larger Raisin Box",
              "caption": "",
              "prompt": "Place the larger object in the grey bin.",
              "note": "Failed"
            },
            {
              "file": "videos/robolab_default/02_FLUX_3_Action/07_marker-in-mug.mp4",
              "thumb": "thumbnails/robolab_default/02_FLUX_3_Action/07_marker-in-mug.webp",
              "title": "Put the Marker in the Mug",
              "caption": "",
              "prompt": "Put the whiteboard marker in the mug",
              "note": "Succeeded"
            },
            {
              "file": "videos/robolab_default/02_FLUX_3_Action/08_white-mugs-upright.mp4",
              "thumb": "thumbnails/robolab_default/02_FLUX_3_Action/08_white-mugs-upright.webp",
              "title": "Stand the White Mugs Upright",
              "caption": "",
              "prompt": "Make sure all the white mugs are upright so that the opening is facing upwards.",
              "note": "Failed"
            },
            {
              "file": "videos/robolab_default/02_FLUX_3_Action/10_clear-the-plate.mp4",
              "thumb": "thumbnails/robolab_default/02_FLUX_3_Action/10_clear-the-plate.webp",
              "title": "Clear the Plate",
              "caption": "",
              "prompt": "Put the two measuring cups outside of the plate",
              "note": "Failed"
            }
          ]
        },
        {
          "name": "π0.5",
          "videos": [
            {
              "file": "videos/robolab_default/03_pi0.5/12_stack-blocks.mp4",
              "thumb": "thumbnails/robolab_default/03_pi0.5/12_stack-blocks.webp",
              "title": "Stack Blocks in Order",
              "caption": "",
              "prompt": "Stack the blocks in the order from bottom to top: red, blue, green, yellow",
              "note": "Failed"
            },
            {
              "file": "videos/robolab_default/03_pi0.5/13_stand-jug-upright.mp4",
              "thumb": "thumbnails/robolab_default/03_pi0.5/13_stand-jug-upright.webp",
              "title": "Stand the Jug Upright",
              "caption": "",
              "prompt": "Stand the jug upright",
              "note": "Failed"
            },
            {
              "file": "videos/robolab_default/03_pi0.5/14_recycle-cartons.mp4",
              "thumb": "thumbnails/robolab_default/03_pi0.5/14_recycle-cartons.webp",
              "title": "Recycle Cartons",
              "caption": "",
              "prompt": "Put the cartons that can be recycled in the vertical crate",
              "note": "Failed"
            },
            {
              "file": "videos/robolab_default/03_pi0.5/15_jugs-on-shelf.mp4",
              "thumb": "thumbnails/robolab_default/03_pi0.5/15_jugs-on-shelf.webp",
              "title": "Put Jugs on the Shelf",
              "caption": "",
              "prompt": "Put all the jugs on the shelf",
              "note": "Failed"
            },
            {
              "file": "videos/robolab_default/03_pi0.5/03_white-and-yellow-objects.mp4",
              "thumb": "thumbnails/robolab_default/03_pi0.5/03_white-and-yellow-objects.webp",
              "title": "Bin the White and Yellow Objects",
              "caption": "",
              "prompt": "Put all white objects and yellow objects in the grey bin",
              "note": "Failed"
            },
            {
              "file": "videos/robolab_default/03_pi0.5/04_mustard-in-left-bin.mp4",
              "thumb": "thumbnails/robolab_default/03_pi0.5/04_mustard-in-left-bin.webp",
              "title": "Put the Mustard in the Left Bin",
              "caption": "",
              "prompt": "Put the mustard in the left bin",
              "note": "Failed"
            },
            {
              "file": "videos/robolab_default/03_pi0.5/05_larger-raisin-box.mp4",
              "thumb": "thumbnails/robolab_default/03_pi0.5/05_larger-raisin-box.webp",
              "title": "Bin the Larger Raisin Box",
              "caption": "",
              "prompt": "Place the larger object in the grey bin.",
              "note": "Failed"
            },
            {
              "file": "videos/robolab_default/03_pi0.5/07_marker-in-mug.mp4",
              "thumb": "thumbnails/robolab_default/03_pi0.5/07_marker-in-mug.webp",
              "title": "Put the Marker in the Mug",
              "caption": "",
              "prompt": "Put the whiteboard marker in the mug",
              "note": "Failed"
            },
            {
              "file": "videos/robolab_default/03_pi0.5/08_white-mugs-upright.mp4",
              "thumb": "thumbnails/robolab_default/03_pi0.5/08_white-mugs-upright.webp",
              "title": "Stand the White Mugs Upright",
              "caption": "",
              "prompt": "Make sure all the white mugs are upright so that the opening is facing upwards.",
              "note": "Succeeded"
            },
            {
              "file": "videos/robolab_default/03_pi0.5/10_clear-the-plate.mp4",
              "thumb": "thumbnails/robolab_default/03_pi0.5/10_clear-the-plate.webp",
              "title": "Clear the Plate",
              "caption": "",
              "prompt": "Put the two measuring cups outside of the plate",
              "note": "Succeeded"
            }
          ]
        }
      ]
    },
    "robolab_specific": {
      "categories": [
        {
          "name": "AMP1",
          "videos": [
            {
              "file": "videos/robolab_specific/01_AMP1/12_stack-blocks.mp4",
              "thumb": "thumbnails/robolab_specific/01_AMP1/12_stack-blocks.webp",
              "title": "Stack Blocks in Order",
              "caption": "",
              "prompt": "Build a tower by placing the red block first, then the blue block on top, then the green, and finally the yellow block on top as a single tower",
              "note": "Succeeded"
            },
            {
              "file": "videos/robolab_specific/01_AMP1/13_stand-jug-upright.mp4",
              "thumb": "thumbnails/robolab_specific/01_AMP1/13_stand-jug-upright.webp",
              "title": "Stand the Jug Upright",
              "caption": "",
              "prompt": "Grasp the jug that is tipped on its side place it upright",
              "note": "Succeeded"
            },
            {
              "file": "videos/robolab_specific/01_AMP1/14_recycle-cartons.mp4",
              "thumb": "thumbnails/robolab_specific/01_AMP1/14_recycle-cartons.webp",
              "title": "Recycle Cartons",
              "caption": "",
              "prompt": "Pick up the food cartons that can be recycled and place them into the vertical crate in the center of the table",
              "note": "Succeeded"
            },
            {
              "file": "videos/robolab_specific/01_AMP1/15_jugs-on-shelf.mp4",
              "thumb": "thumbnails/robolab_specific/01_AMP1/15_jugs-on-shelf.webp",
              "title": "Put Jugs on the Shelf",
              "caption": "",
              "prompt": "Pick up both jugs from the table and place them on the shelf",
              "note": "Succeeded"
            },
            {
              "file": "videos/robolab_specific/01_AMP1/03_white-and-yellow-objects.mp4",
              "thumb": "thumbnails/robolab_specific/01_AMP1/03_white-and-yellow-objects.webp",
              "title": "Bin the White and Yellow Objects",
              "caption": "",
              "prompt": "Identify the white-colored and yellow-colored object on the table and place each one into the grey bin",
              "note": "Succeeded"
            },
            {
              "file": "videos/robolab_specific/01_AMP1/04_mustard-in-left-bin.mp4",
              "thumb": "thumbnails/robolab_specific/01_AMP1/04_mustard-in-left-bin.webp",
              "title": "Put the Mustard in the Left Bin",
              "caption": "",
              "prompt": "Pick up the mustard bottle and place it in the bin on the left side",
              "note": "Succeeded"
            },
            {
              "file": "videos/robolab_specific/01_AMP1/05_larger-raisin-box.mp4",
              "thumb": "thumbnails/robolab_specific/01_AMP1/05_larger-raisin-box.webp",
              "title": "Bin the Larger Raisin Box",
              "caption": "",
              "prompt": "Compare the objects and put the larger raisin box in the grey bin",
              "note": "Succeeded"
            },
            {
              "file": "videos/robolab_specific/01_AMP1/07_marker-in-mug.mp4",
              "thumb": "thumbnails/robolab_specific/01_AMP1/07_marker-in-mug.webp",
              "title": "Put the Marker in the Mug",
              "caption": "",
              "prompt": "Pick up the marker and drop it vertically into the mug, make sure the marker stands upright",
              "note": "Succeeded"
            },
            {
              "file": "videos/robolab_specific/01_AMP1/08_white-mugs-upright.mp4",
              "thumb": "thumbnails/robolab_specific/01_AMP1/08_white-mugs-upright.webp",
              "title": "Stand the White Mugs Upright",
              "caption": "",
              "prompt": "For each white mug that is tipped over or upside down, rotate it so the opening faces straight up",
              "note": "Succeeded"
            },
            {
              "file": "videos/robolab_specific/01_AMP1/10_clear-the-plate.mp4",
              "thumb": "thumbnails/robolab_specific/01_AMP1/10_clear-the-plate.webp",
              "title": "Clear the Plate",
              "caption": "",
              "prompt": "Pick up the orange measuring cup and the blue measuring cup from the plate and place each on the table surface",
              "note": "Succeeded"
            }
          ]
        },
        {
          "name": "FLUX 3 Action",
          "videos": [
            {
              "file": "videos/robolab_specific/02_FLUX_3_Action/12_stack-blocks.mp4",
              "thumb": "thumbnails/robolab_specific/02_FLUX_3_Action/12_stack-blocks.webp",
              "title": "Stack Blocks in Order",
              "caption": "",
              "prompt": "Build a tower by placing the red block first, then the blue block on top, then the green, and finally the yellow block on top as a single tower",
              "note": "Failed"
            },
            {
              "file": "videos/robolab_specific/02_FLUX_3_Action/13_stand-jug-upright.mp4",
              "thumb": "thumbnails/robolab_specific/02_FLUX_3_Action/13_stand-jug-upright.webp",
              "title": "Stand the Jug Upright",
              "caption": "",
              "prompt": "Grasp the jug that is tipped on its side place it upright",
              "note": "Failed"
            },
            {
              "file": "videos/robolab_specific/02_FLUX_3_Action/14_recycle-cartons.mp4",
              "thumb": "thumbnails/robolab_specific/02_FLUX_3_Action/14_recycle-cartons.webp",
              "title": "Recycle Cartons",
              "caption": "",
              "prompt": "Pick up the food cartons that can be recycled and place them into the vertical crate in the center of the table",
              "note": "Failed"
            },
            {
              "file": "videos/robolab_specific/02_FLUX_3_Action/15_jugs-on-shelf.mp4",
              "thumb": "thumbnails/robolab_specific/02_FLUX_3_Action/15_jugs-on-shelf.webp",
              "title": "Put Jugs on the Shelf",
              "caption": "",
              "prompt": "Pick up both jugs from the table and place them on the shelf",
              "note": "Failed"
            },
            {
              "file": "videos/robolab_specific/02_FLUX_3_Action/03_white-and-yellow-objects.mp4",
              "thumb": "thumbnails/robolab_specific/02_FLUX_3_Action/03_white-and-yellow-objects.webp",
              "title": "Bin the White and Yellow Objects",
              "caption": "",
              "prompt": "Identify the white-colored and yellow-colored object on the table and place each one into the grey bin",
              "note": "Succeeded"
            },
            {
              "file": "videos/robolab_specific/02_FLUX_3_Action/04_mustard-in-left-bin.mp4",
              "thumb": "thumbnails/robolab_specific/02_FLUX_3_Action/04_mustard-in-left-bin.webp",
              "title": "Put the Mustard in the Left Bin",
              "caption": "",
              "prompt": "Pick up the mustard bottle and place it in the bin on the left side",
              "note": "Succeeded"
            },
            {
              "file": "videos/robolab_specific/02_FLUX_3_Action/05_larger-raisin-box.mp4",
              "thumb": "thumbnails/robolab_specific/02_FLUX_3_Action/05_larger-raisin-box.webp",
              "title": "Bin the Larger Raisin Box",
              "caption": "",
              "prompt": "Compare the objects and put the larger raisin box in the grey bin",
              "note": "Failed"
            },
            {
              "file": "videos/robolab_specific/02_FLUX_3_Action/07_marker-in-mug.mp4",
              "thumb": "thumbnails/robolab_specific/02_FLUX_3_Action/07_marker-in-mug.webp",
              "title": "Put the Marker in the Mug",
              "caption": "",
              "prompt": "Pick up the marker and drop it vertically into the mug, make sure the marker stands upright",
              "note": "Failed"
            },
            {
              "file": "videos/robolab_specific/02_FLUX_3_Action/08_white-mugs-upright.mp4",
              "thumb": "thumbnails/robolab_specific/02_FLUX_3_Action/08_white-mugs-upright.webp",
              "title": "Stand the White Mugs Upright",
              "caption": "",
              "prompt": "For each white mug that is tipped over or upside down, rotate it so the opening faces straight up",
              "note": "Failed"
            },
            {
              "file": "videos/robolab_specific/02_FLUX_3_Action/10_clear-the-plate.mp4",
              "thumb": "thumbnails/robolab_specific/02_FLUX_3_Action/10_clear-the-plate.webp",
              "title": "Clear the Plate",
              "caption": "",
              "prompt": "Pick up the orange measuring cup and the blue measuring cup from the plate and place each on the table surface",
              "note": "Succeeded"
            }
          ]
        },
        {
          "name": "π0.5",
          "videos": [
            {
              "file": "videos/robolab_specific/03_pi0.5/12_stack-blocks.mp4",
              "thumb": "thumbnails/robolab_specific/03_pi0.5/12_stack-blocks.webp",
              "title": "Stack Blocks in Order",
              "caption": "",
              "prompt": "Build a tower by placing the red block first, then the blue block on top, then the green, and finally the yellow block on top as a single tower",
              "note": "Failed"
            },
            {
              "file": "videos/robolab_specific/03_pi0.5/13_stand-jug-upright.mp4",
              "thumb": "thumbnails/robolab_specific/03_pi0.5/13_stand-jug-upright.webp",
              "title": "Stand the Jug Upright",
              "caption": "",
              "prompt": "Grasp the jug that is tipped on its side place it upright",
              "note": "Failed"
            },
            {
              "file": "videos/robolab_specific/03_pi0.5/14_recycle-cartons.mp4",
              "thumb": "thumbnails/robolab_specific/03_pi0.5/14_recycle-cartons.webp",
              "title": "Recycle Cartons",
              "caption": "",
              "prompt": "Pick up the food cartons that can be recycled and place them into the vertical crate in the center of the table",
              "note": "Failed"
            },
            {
              "file": "videos/robolab_specific/03_pi0.5/15_jugs-on-shelf.mp4",
              "thumb": "thumbnails/robolab_specific/03_pi0.5/15_jugs-on-shelf.webp",
              "title": "Put Jugs on the Shelf",
              "caption": "",
              "prompt": "Pick up both jugs from the table and place them on the shelf",
              "note": "Failed"
            },
            {
              "file": "videos/robolab_specific/03_pi0.5/03_white-and-yellow-objects.mp4",
              "thumb": "thumbnails/robolab_specific/03_pi0.5/03_white-and-yellow-objects.webp",
              "title": "Bin the White and Yellow Objects",
              "caption": "",
              "prompt": "Identify the white-colored and yellow-colored object on the table and place each one into the grey bin",
              "note": "Failed"
            },
            {
              "file": "videos/robolab_specific/03_pi0.5/04_mustard-in-left-bin.mp4",
              "thumb": "thumbnails/robolab_specific/03_pi0.5/04_mustard-in-left-bin.webp",
              "title": "Put the Mustard in the Left Bin",
              "caption": "",
              "prompt": "Pick up the mustard bottle and place it in the bin on the left side",
              "note": "Succeeded"
            },
            {
              "file": "videos/robolab_specific/03_pi0.5/05_larger-raisin-box.mp4",
              "thumb": "thumbnails/robolab_specific/03_pi0.5/05_larger-raisin-box.webp",
              "title": "Bin the Larger Raisin Box",
              "caption": "",
              "prompt": "Compare the objects and put the larger raisin box in the grey bin",
              "note": "Succeeded"
            },
            {
              "file": "videos/robolab_specific/03_pi0.5/07_marker-in-mug.mp4",
              "thumb": "thumbnails/robolab_specific/03_pi0.5/07_marker-in-mug.webp",
              "title": "Put the Marker in the Mug",
              "caption": "",
              "prompt": "Pick up the marker and drop it vertically into the mug, make sure the marker stands upright",
              "note": "Failed"
            },
            {
              "file": "videos/robolab_specific/03_pi0.5/08_white-mugs-upright.mp4",
              "thumb": "thumbnails/robolab_specific/03_pi0.5/08_white-mugs-upright.webp",
              "title": "Stand the White Mugs Upright",
              "caption": "",
              "prompt": "For each white mug that is tipped over or upside down, rotate it so the opening faces straight up",
              "note": "Failed"
            },
            {
              "file": "videos/robolab_specific/03_pi0.5/10_clear-the-plate.mp4",
              "thumb": "thumbnails/robolab_specific/03_pi0.5/10_clear-the-plate.webp",
              "title": "Clear the Plate",
              "caption": "",
              "prompt": "Pick up the orange measuring cup and the blue measuring cup from the plate and place each on the table surface",
              "note": "Succeeded"
            }
          ]
        }
      ]
    },
    "robolab_vague": {
      "categories": [
        {
          "name": "AMP1",
          "videos": [
            {
              "file": "videos/robolab_vague/01_AMP1/12_stack-blocks.mp4",
              "thumb": "thumbnails/robolab_vague/01_AMP1/12_stack-blocks.webp",
              "title": "Stack Blocks in Order",
              "caption": "",
              "prompt": "Stack in the order of red, blue, green, yellow",
              "note": "Succeeded"
            },
            {
              "file": "videos/robolab_vague/01_AMP1/13_stand-jug-upright.mp4",
              "thumb": "thumbnails/robolab_vague/01_AMP1/13_stand-jug-upright.webp",
              "title": "Stand the Jug Upright",
              "caption": "",
              "prompt": "Fix the jug",
              "note": "Succeeded"
            },
            {
              "file": "videos/robolab_vague/01_AMP1/14_recycle-cartons.mp4",
              "thumb": "thumbnails/robolab_vague/01_AMP1/14_recycle-cartons.webp",
              "title": "Recycle Cartons",
              "caption": "",
              "prompt": "Recycle the cartons in the crate",
              "note": "Succeeded"
            },
            {
              "file": "videos/robolab_vague/01_AMP1/15_jugs-on-shelf.mp4",
              "thumb": "thumbnails/robolab_vague/01_AMP1/15_jugs-on-shelf.webp",
              "title": "Put Jugs on the Shelf",
              "caption": "",
              "prompt": "Put the jugs on the shelf",
              "note": "Succeeded"
            },
            {
              "file": "videos/robolab_vague/01_AMP1/03_white-and-yellow-objects.mp4",
              "thumb": "thumbnails/robolab_vague/01_AMP1/03_white-and-yellow-objects.webp",
              "title": "Bin the White and Yellow Objects",
              "caption": "",
              "prompt": "Clean up white and yellow objects",
              "note": "Succeeded"
            },
            {
              "file": "videos/robolab_vague/01_AMP1/04_mustard-in-left-bin.mp4",
              "thumb": "thumbnails/robolab_vague/01_AMP1/04_mustard-in-left-bin.webp",
              "title": "Put the Mustard in the Left Bin",
              "caption": "",
              "prompt": "Use the left bin for mustard",
              "note": "Succeeded"
            },
            {
              "file": "videos/robolab_vague/01_AMP1/05_larger-raisin-box.mp4",
              "thumb": "thumbnails/robolab_vague/01_AMP1/05_larger-raisin-box.webp",
              "title": "Bin the Larger Raisin Box",
              "caption": "",
              "prompt": "Put the larger object away",
              "note": "Succeeded"
            },
            {
              "file": "videos/robolab_vague/01_AMP1/07_marker-in-mug.mp4",
              "thumb": "thumbnails/robolab_vague/01_AMP1/07_marker-in-mug.webp",
              "title": "Put the Marker in the Mug",
              "caption": "",
              "prompt": "Put the marker in the mug",
              "note": "Succeeded"
            },
            {
              "file": "videos/robolab_vague/01_AMP1/08_white-mugs-upright.mp4",
              "thumb": "thumbnails/robolab_vague/01_AMP1/08_white-mugs-upright.webp",
              "title": "Stand the White Mugs Upright",
              "caption": "",
              "prompt": "Fix the cups",
              "note": "Succeeded"
            },
            {
              "file": "videos/robolab_vague/01_AMP1/10_clear-the-plate.mp4",
              "thumb": "thumbnails/robolab_vague/01_AMP1/10_clear-the-plate.webp",
              "title": "Clear the Plate",
              "caption": "",
              "prompt": "Clear the plate",
              "note": "Succeeded"
            }
          ]
        },
        {
          "name": "FLUX 3 Action",
          "videos": [
            {
              "file": "videos/robolab_vague/02_FLUX_3_Action/12_stack-blocks.mp4",
              "thumb": "thumbnails/robolab_vague/02_FLUX_3_Action/12_stack-blocks.webp",
              "title": "Stack Blocks in Order",
              "caption": "",
              "prompt": "Stack in the order of red, blue, green, yellow",
              "note": "Failed"
            },
            {
              "file": "videos/robolab_vague/02_FLUX_3_Action/13_stand-jug-upright.mp4",
              "thumb": "thumbnails/robolab_vague/02_FLUX_3_Action/13_stand-jug-upright.webp",
              "title": "Stand the Jug Upright",
              "caption": "",
              "prompt": "Fix the jug",
              "note": "Succeeded"
            },
            {
              "file": "videos/robolab_vague/02_FLUX_3_Action/14_recycle-cartons.mp4",
              "thumb": "thumbnails/robolab_vague/02_FLUX_3_Action/14_recycle-cartons.webp",
              "title": "Recycle Cartons",
              "caption": "",
              "prompt": "Recycle the cartons in the crate",
              "note": "Failed"
            },
            {
              "file": "videos/robolab_vague/02_FLUX_3_Action/15_jugs-on-shelf.mp4",
              "thumb": "thumbnails/robolab_vague/02_FLUX_3_Action/15_jugs-on-shelf.webp",
              "title": "Put Jugs on the Shelf",
              "caption": "",
              "prompt": "Put the jugs on the shelf",
              "note": "Failed"
            },
            {
              "file": "videos/robolab_vague/02_FLUX_3_Action/03_white-and-yellow-objects.mp4",
              "thumb": "thumbnails/robolab_vague/02_FLUX_3_Action/03_white-and-yellow-objects.webp",
              "title": "Bin the White and Yellow Objects",
              "caption": "",
              "prompt": "Clean up white and yellow objects",
              "note": "Failed"
            },
            {
              "file": "videos/robolab_vague/02_FLUX_3_Action/04_mustard-in-left-bin.mp4",
              "thumb": "thumbnails/robolab_vague/02_FLUX_3_Action/04_mustard-in-left-bin.webp",
              "title": "Put the Mustard in the Left Bin",
              "caption": "",
              "prompt": "Use the left bin for mustard",
              "note": "Failed"
            },
            {
              "file": "videos/robolab_vague/02_FLUX_3_Action/05_larger-raisin-box.mp4",
              "thumb": "thumbnails/robolab_vague/02_FLUX_3_Action/05_larger-raisin-box.webp",
              "title": "Bin the Larger Raisin Box",
              "caption": "",
              "prompt": "Put the larger object away",
              "note": "Failed"
            },
            {
              "file": "videos/robolab_vague/02_FLUX_3_Action/07_marker-in-mug.mp4",
              "thumb": "thumbnails/robolab_vague/02_FLUX_3_Action/07_marker-in-mug.webp",
              "title": "Put the Marker in the Mug",
              "caption": "",
              "prompt": "Put the marker in the mug",
              "note": "Failed"
            },
            {
              "file": "videos/robolab_vague/02_FLUX_3_Action/08_white-mugs-upright.mp4",
              "thumb": "thumbnails/robolab_vague/02_FLUX_3_Action/08_white-mugs-upright.webp",
              "title": "Stand the White Mugs Upright",
              "caption": "",
              "prompt": "Fix the cups",
              "note": "Failed"
            },
            {
              "file": "videos/robolab_vague/02_FLUX_3_Action/10_clear-the-plate.mp4",
              "thumb": "thumbnails/robolab_vague/02_FLUX_3_Action/10_clear-the-plate.webp",
              "title": "Clear the Plate",
              "caption": "",
              "prompt": "Clear the plate",
              "note": "Failed"
            }
          ]
        },
        {
          "name": "π0.5",
          "videos": [
            {
              "file": "videos/robolab_vague/03_pi0.5/12_stack-blocks.mp4",
              "thumb": "thumbnails/robolab_vague/03_pi0.5/12_stack-blocks.webp",
              "title": "Stack Blocks in Order",
              "caption": "",
              "prompt": "Stack in the order of red, blue, green, yellow",
              "note": "Failed"
            },
            {
              "file": "videos/robolab_vague/03_pi0.5/13_stand-jug-upright.mp4",
              "thumb": "thumbnails/robolab_vague/03_pi0.5/13_stand-jug-upright.webp",
              "title": "Stand the Jug Upright",
              "caption": "",
              "prompt": "Fix the jug",
              "note": "Failed"
            },
            {
              "file": "videos/robolab_vague/03_pi0.5/14_recycle-cartons.mp4",
              "thumb": "thumbnails/robolab_vague/03_pi0.5/14_recycle-cartons.webp",
              "title": "Recycle Cartons",
              "caption": "",
              "prompt": "Recycle the cartons in the crate",
              "note": "Failed"
            },
            {
              "file": "videos/robolab_vague/03_pi0.5/15_jugs-on-shelf.mp4",
              "thumb": "thumbnails/robolab_vague/03_pi0.5/15_jugs-on-shelf.webp",
              "title": "Put Jugs on the Shelf",
              "caption": "",
              "prompt": "Put the jugs on the shelf",
              "note": "Failed"
            },
            {
              "file": "videos/robolab_vague/03_pi0.5/03_white-and-yellow-objects.mp4",
              "thumb": "thumbnails/robolab_vague/03_pi0.5/03_white-and-yellow-objects.webp",
              "title": "Bin the White and Yellow Objects",
              "caption": "",
              "prompt": "Clean up white and yellow objects",
              "note": "Failed"
            },
            {
              "file": "videos/robolab_vague/03_pi0.5/04_mustard-in-left-bin.mp4",
              "thumb": "thumbnails/robolab_vague/03_pi0.5/04_mustard-in-left-bin.webp",
              "title": "Put the Mustard in the Left Bin",
              "caption": "",
              "prompt": "Use the left bin for mustard",
              "note": "Failed"
            },
            {
              "file": "videos/robolab_vague/03_pi0.5/05_larger-raisin-box.mp4",
              "thumb": "thumbnails/robolab_vague/03_pi0.5/05_larger-raisin-box.webp",
              "title": "Bin the Larger Raisin Box",
              "caption": "",
              "prompt": "Put the larger object away",
              "note": "Failed"
            },
            {
              "file": "videos/robolab_vague/03_pi0.5/07_marker-in-mug.mp4",
              "thumb": "thumbnails/robolab_vague/03_pi0.5/07_marker-in-mug.webp",
              "title": "Put the Marker in the Mug",
              "caption": "",
              "prompt": "Put the marker in the mug",
              "note": "Failed"
            },
            {
              "file": "videos/robolab_vague/03_pi0.5/08_white-mugs-upright.mp4",
              "thumb": "thumbnails/robolab_vague/03_pi0.5/08_white-mugs-upright.webp",
              "title": "Stand the White Mugs Upright",
              "caption": "",
              "prompt": "Fix the cups",
              "note": "Failed"
            },
            {
              "file": "videos/robolab_vague/03_pi0.5/10_clear-the-plate.mp4",
              "thumb": "thumbnails/robolab_vague/03_pi0.5/10_clear-the-plate.webp",
              "title": "Clear the Plate",
              "caption": "",
              "prompt": "Clear the plate",
              "note": "Failed"
            }
          ]
        }
      ]
    }
  }
};
