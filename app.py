from __future__ import annotations

import json
import sys
import tempfile
from pathlib import Path
from typing import Optional

from flask import Flask, Response, request
from werkzeug.utils import secure_filename


THIS_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(THIS_DIR))

import stage4  # noqa: E402  (local file import)


app = Flask(__name__)


def _json_response(payload: dict, status: int) -> Response:
    res = Response(
        json.dumps(payload, ensure_ascii=False),
        status=status,
        mimetype="application/json",
    )
    res.headers["Access-Control-Allow-Origin"] = "*"
    res.headers["Access-Control-Allow-Methods"] = "POST, OPTIONS"
    res.headers["Access-Control-Allow-Headers"] = "Content-Type"
    return res


def _text_response(text: str, status: int, *, filename: Optional[str] = None) -> Response:
    res = Response(
        text,
        status=status,
        mimetype="text/plain; charset=utf-8",
    )
    res.headers["Access-Control-Allow-Origin"] = "*"
    res.headers["Access-Control-Allow-Methods"] = "GET, OPTIONS"
    res.headers["Access-Control-Allow-Headers"] = "Content-Type"
    if filename:
        res.headers["Content-Disposition"] = f'attachment; filename="{filename}"'
    return res


@app.route("/rerouter-script", methods=["GET", "OPTIONS"])
def rerouter_script() -> Response:
    if request.method == "OPTIONS":
        return _text_response("", 204)

    script_path = THIS_DIR / "bim_auto_rerouter_v5_1.txt"
    if not script_path.exists():
        return _text_response("Missing script file: bim_auto_rerouter_v5_1.txt", 404)

    return _text_response(
        script_path.read_text(encoding="utf-8"),
        200,
        filename="bim_auto_rerouter_v5_1.txt",
    )


@app.route("/convert-xml", methods=["POST", "OPTIONS"])
def convert_xml() -> Response:
    if request.method == "OPTIONS":
        return _json_response({}, 204)

    upload = request.files.get("file")
    if upload is None:
        return _json_response({"error": "Missing file upload field 'file'."}, 400)

    original_name = upload.filename or ""
    safe_name = secure_filename(original_name) or "upload.xml"
    if not safe_name.lower().endswith(".xml"):
        safe_name = f"{safe_name}.xml"

    try:
        with tempfile.TemporaryDirectory(prefix="stage4_") as tmpdir:
            tmpdir_path = Path(tmpdir)
            xml_path = tmpdir_path / safe_name
            xml_path.write_bytes(upload.read())

            output_path = tmpdir_path / "clash_rule_engine_report.json"
            output = stage4.run(str(xml_path), str(output_path))

        return _json_response(output, 200)

    except Exception as exc:  # noqa: BLE001
        return _json_response({"error": str(exc)}, 500)


if __name__ == "__main__":
    # Runs locally at http://127.0.0.1:5000
    app.run(host="127.0.0.1", port=5000, debug=True)
