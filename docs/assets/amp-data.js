//
// For licensing see accompanying LICENSE file.
// Copyright (C) 2026 Apple Inc. All Rights Reserved.
//
const AMP_DATA = {
 "realworld": {
  "note": "12 real-world tasks, 9 trials each: AMP1 succeeds in 104 of 108 trials, FLUX 3 Action in 49, and π0.5 in 30.",
  "entries": [
   {
    "name": "AMP1",
    "kind": "ours",
    "v": {
     "default": 96.3
    }
   },
   {
    "name": "FLUX 3 Action",
    "kind": "wam",
    "v": {
     "default": 45.4
    }
   },
   {
    "name": "π0.5",
    "kind": "vla",
    "v": {
     "default": 27.8
    }
   }
  ]
 },
 "benchmarks": {
  "RoboLab-120": {
   "note": "* Not on the official leaderboard: Astranomy was evaluated outside it, over five episodes per task; the best leaderboard entry is SenseNova-RoboRSI (57.1%). AMP1: 1,200 episodes per instruction condition (10 per task), as in the paper.",
   "conditions": [
    "default",
    "vague",
    "specific"
   ],
   "footnote": "* Not on the official leaderboard: Astranomy was evaluated outside it, over five episodes per task; the best leaderboard entry is SenseNova-RoboRSI (57.1%).",
   "entries": [
    {
     "name": "AMP1 (Gemini 4 Argon)",
     "kind": "ours",
     "v": {
      "default": 85.6,
      "vague": 84,
      "specific": 87.1
     }
    },
    {
     "name": "AMP1 (Claude Opus 5.5)",
     "kind": "ours",
     "v": {
      "default": 82.8,
      "vague": 79.8,
      "specific": 82.8
     }
    },
    {
     "name": "Astranomy (GPT-6 Astra)*",
     "kind": "agent",
     "v": {
      "default": 74.8
     }
    },
    {
     "name": "SenseNova-RoboRSI (GPT-6 Astra)",
     "kind": "agent",
     "v": {
      "default": 57.1
     }
    },
    {
     "name": "Lakeside",
     "kind": "wam",
     "v": {
      "default": 51
     }
    },
    {
     "name": "BiMind v0.5 (strongest VLA)",
     "kind": "vla",
     "v": {
      "default": 49.2,
      "vague": 46.2,
      "specific": 47.2
     }
    },
    {
     "name": "FLUX 3 Action",
     "kind": "wam",
     "v": {
      "default": 40.2,
      "vague": 28.4,
      "specific": 41.9
     }
    },
    {
     "name": "π0.5",
     "kind": "vla",
     "v": {
      "default": 28,
      "vague": 15.2,
      "specific": 28.1
     }
    }
   ]
  },
  "LIBERO-PRO": {
   "note": "3,000 episodes (50 per task). HarnessPAI evolves its program on seeds inside the evaluation (few-shot); AMP1 solves every episode zero-shot.",
   "entries": [
    {
     "name": "HarnessPAI",
     "kind": "agentvla",
     "regime": "fewshot",
     "v": {
      "default": 96.5
     }
    },
    {
     "name": "AMP1 (Gemini 4 Argon)",
     "kind": "ours",
     "v": {
      "default": 95.2
     }
    },
    {
     "name": "AMP1 (Claude Opus 5.5)",
     "kind": "ours",
     "v": {
      "default": 93.2
     }
    },
    {
     "name": "Harness VLA (CC)",
     "kind": "agentvla",
     "regime": "fewshot",
     "v": {
      "default": 87.7
     }
    },
    {
     "name": "Harness VLA (Codex)",
     "kind": "agentvla",
     "regime": "fewshot",
     "v": {
      "default": 79.3
     }
    },
    {
     "name": "ASPIRE",
     "kind": "agent",
     "regime": "fewshot",
     "v": {
      "default": 71.7
     }
    },
    {
     "name": "π0.5 (fine-tuned VLA)",
     "kind": "vla",
     "regime": "finetuned",
     "v": {
      "default": 34.9
     }
    },
    {
     "name": "CaP-Agent0 (zero-shot agent)",
     "kind": "agent",
     "v": {
      "default": 18.2
     }
    },
    {
     "name": "OpenVLA (fine-tuned)",
     "kind": "vla",
     "regime": "finetuned",
     "v": {
      "default": 0
     }
    },
    {
     "name": "π0 (fine-tuned)",
     "kind": "vla",
     "regime": "finetuned",
     "v": {
      "default": 0
     }
    }
   ]
  },
  "robosuite": {
   "note": "700 episodes (100 per task), including two tasks that coordinate two arms.",
   "entries": [
    {
     "name": "AMP1 (Gemini 4 Argon)",
     "kind": "ours",
     "v": {
      "default": 100
     }
    },
    {
     "name": "AMP1 (Claude Opus 5.5)",
     "kind": "ours",
     "v": {
      "default": 98.4
     }
    },
    {
     "name": "HarnessPAI",
     "kind": "agent",
     "regime": "fewshot",
     "v": {
      "default": 96.3
     }
    },
    {
     "name": "ASPIRE",
     "kind": "agent",
     "regime": "fewshot",
     "v": {
      "default": 81
     }
    },
    {
     "name": "CaP-Agent0",
     "kind": "agent",
     "regime": "fewshot",
     "v": {
      "default": 68.3
     }
    },
    {
     "name": "Human-expert programs",
     "kind": "human",
     "v": {
      "default": 82.6
     }
    }
   ]
  },
  "BEHAVIOR-1K": {
   "note": "Two mobile-manipulation tasks, 50 episodes (25 per task). ASPIRE repairs its programs over rollouts of the same tasks.",
   "entries": [
    {
     "name": "ASPIRE (Opus 5.5)",
     "kind": "agent",
     "regime": "fewshot",
     "v": {
      "default": 100
     }
    },
    {
     "name": "AMP1 (Gemini 4 Argon)",
     "kind": "ours",
     "v": {
      "default": 96
     }
    },
    {
     "name": "AMP1 (Claude Opus 5.5)",
     "kind": "ours",
     "v": {
      "default": 96
     }
    },
    {
     "name": "HarnessPAI",
     "kind": "agent",
     "regime": "fewshot",
     "v": {
      "default": 96
     }
    },
    {
     "name": "ASPIRE (GPT-6 Astra)",
     "kind": "agent",
     "regime": "fewshot",
     "v": {
      "default": 96
     }
    },
    {
     "name": "ASPIRE (Opus 4.6)",
     "kind": "agent",
     "regime": "fewshot",
     "v": {
      "default": 88
     }
    },
    {
     "name": "CaP-Agent0",
     "kind": "agent",
     "v": {
      "default": 64
     }
    },
    {
     "name": "Human-written programs",
     "kind": "human",
     "v": {
      "default": 54
     }
    }
   ]
  },
  "RoboDojo": {
   "note": "",
   "entries": [
    {
     "name": "AMP1+",
     "kind": "ours",
     "regime": "fewshot",
     "v": {
      "default": 84.1
     }
    },
    {
     "name": "Q0-Preview",
     "kind": "agent",
     "regime": "fewshot",
     "v": {
      "default": 68.9
     }
    },
    {
     "name": "GO-3 Nano Preview",
     "kind": "agentwam",
     "regime": "fewshot",
     "v": {
      "default": 56.4
     }
    },
    {
     "name": "PyRUA-Lean",
     "kind": "agent",
     "regime": "fewshot",
     "v": {
      "default": 50.4
     }
    },
    {
     "name": "HexaShell-Preview",
     "kind": "agent",
     "regime": "fewshot",
     "v": {
      "default": 46.9
     }
    },
    {
     "name": "WorldScape-R²SI",
     "kind": "agentwam",
     "regime": "fewshot",
     "v": {
      "default": 39.4
     }
    },
    {
     "name": "EmbodiedEvo",
     "kind": "agent",
     "regime": "fewshot",
     "v": {
      "default": 39.0
     }
    },
    {
     "name": "WALL-OSS-1.0-lite",
     "kind": "vla",
     "regime": "finetuned",
     "v": {
      "default": 35.0
     }
    }
   ]
  }
 }
};
