"""
AI pipeline: audio -> Whisper Tiny (ASR) -> Phi-3.5 Mini via Ollama (LLM) -> raw intent JSON.

This module is deliberately ROS-free and blocking. The ROS node runs it in a
worker thread (never on the executor), so however slow it is, telemetry,
feedback, safety and cancellation keep working.

The LLM only produces a *semantic* intent in human terms. It does NOT convert
units or pick sign conventions; that is the deterministic normaliser's job.
"""
import time

import requests
from faster_whisper import WhisperModel

SYSTEM_PROMPT = """You convert a spoken robot command into JSON.
Output ONLY one JSON object with exactly these keys:
"action": one of "move", "turn", "stop", "unsupported"
"distance": number or null
"distance_unit": the unit word as spoken (e.g. "meters", "feet") or null
"direction": "forward", "backward", "left", "right" or null
"angle": number or null
"angle_unit": the unit word as spoken (e.g. "degrees") or null
Do NOT convert units. Use "unsupported" for anything that is not a move, turn or stop.
Examples:
"Move forward 10 meters." -> {"action": "move", "distance": 10, "distance_unit": "meters", "direction": "forward", "angle": null, "angle_unit": null}
"Turn 45 degrees right." -> {"action": "turn", "distance": null, "distance_unit": null, "direction": "right", "angle": 45, "angle_unit": "degrees"}
"Stop." -> {"action": "stop", "distance": null, "distance_unit": null, "direction": null, "angle": null, "angle_unit": null}
"Sing a song." -> {"action": "unsupported", "distance": null, "distance_unit": null, "direction": null, "angle": null, "angle_unit": null}"""


class AIPipeline:
    def __init__(self, whisper_model="tiny.en", llm_model="phi3.5",
                 ollama_url="http://localhost:11434", simulated_latency_s=0.0):
        # CPU int8: fast enough for Whisper Tiny, keeps the GPU free for the LLM
        self.asr = WhisperModel(whisper_model, device="cpu", compute_type="int8")
        self.llm_model = llm_model
        self.ollama_url = ollama_url
        self.simulated_latency_s = simulated_latency_s

    def transcribe(self, wav_path: str) -> str:
        segments, _ = self.asr.transcribe(wav_path, beam_size=1, language="en")
        return " ".join(s.text.strip() for s in segments).strip()

    def interpret(self, text: str) -> str:
        resp = requests.post(
            f"{self.ollama_url}/api/generate",
            json={
                "model": self.llm_model,
                "system": SYSTEM_PROMPT,
                "prompt": text,
                "format": "json",          # constrain output to JSON
                "stream": False,
                "options": {"temperature": 0},
            },
            timeout=60,
        )
        resp.raise_for_status()
        return resp.json()["response"]

    def warmup(self):
        """Load the LLM into memory so the first real command isn't slow."""
        self.interpret("Stop.")

    def process(self, wav_path: str) -> dict:
        t0 = time.monotonic()
        transcript = self.transcribe(wav_path)
        t1 = time.monotonic()
        raw = self.interpret(transcript)
        t2 = time.monotonic()
        if self.simulated_latency_s > 0:
            time.sleep(self.simulated_latency_s)   # injected delay for Scenario 3
        t3 = time.monotonic()
        return {
            "transcript": transcript,
            "llm_raw": raw,
            "asr_s": t1 - t0,
            "llm_s": t2 - t1,
            "injected_s": t3 - t2,
            "total_s": t3 - t0,
        }


if __name__ == "__main__":
    # Standalone test: run every .wav in a folder through ASR -> LLM -> normaliser.
    import glob
    import os
    import sys
    from normaliser import normalise, NormalisationError

    folder = sys.argv[1] if len(sys.argv) > 1 else "."
    pipe = AIPipeline()
    print("Warming up LLM...")
    pipe.warmup()
    for wav in sorted(glob.glob(os.path.join(folder, "*.wav"))):
        out = pipe.process(wav)
        try:
            cmd = f"OK -> {normalise(out['llm_raw'])}"
        except NormalisationError as e:
            cmd = f"REJECT -> {e}"
        print(f"\n[{os.path.basename(wav)}]")
        print(f"  transcript : {out['transcript']!r}")
        print(f"  llm_raw    : {out['llm_raw'].strip()}")
        print(f"  normalised : {cmd}")
        print(f"  timing     : asr={out['asr_s']:.2f}s llm={out['llm_s']:.2f}s")