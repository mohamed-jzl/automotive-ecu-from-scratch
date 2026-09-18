"""
HTML test report generator.

Why a report file rather than just console output
-------------------------------------------------
Console output scrolls away. A test report is a record: it says which build was
tested, on what equipment, when, and what the outcome was for each requirement.
In a regulated development process that record is a deliverable, and "it worked
on my bench last Tuesday" is not.

The report is deliberately self-contained - CSS inlined, no external fonts, no
JavaScript - so it can be attached to an email, committed to the repository, or
opened on a machine with no network. A report that needs a CDN to render is a
report that stops rendering the moment it matters.
"""

from __future__ import annotations

import html
from datetime import datetime
from typing import Any

from test_cases import ERROR, FAIL, PASS, SKIP, TestResult

_STATUS_COLOUR = {
    PASS:  "#1a7f37",
    FAIL:  "#cf222e",
    SKIP:  "#9a6700",
    ERROR: "#8250df",
}

_CSS = """
:root{--bg:#ffffff;--fg:#1f2328;--muted:#656d76;--border:#d1d9e0;--panel:#f6f8fa}
@media (prefers-color-scheme:dark){
  :root{--bg:#0d1117;--fg:#e6edf3;--muted:#9198a1;--border:#30363d;--panel:#161b22}
}
*{box-sizing:border-box}
body{margin:0;padding:2rem 1.25rem;background:var(--bg);color:var(--fg);
     font:14px/1.6 -apple-system,BlinkMacSystemFont,"Segoe UI",Helvetica,Arial,sans-serif}
.wrap{max-width:1000px;margin:0 auto}
h1{font-size:1.5rem;margin:0 0 .25rem}
.sub{color:var(--muted);margin:0 0 1.5rem}
.cards{display:flex;flex-wrap:wrap;gap:.75rem;margin-bottom:1.5rem}
.card{flex:1 1 130px;border:1px solid var(--border);border-radius:8px;
      padding:.75rem 1rem;background:var(--panel)}
.card .n{font-size:1.6rem;font-weight:600;line-height:1.2}
.card .l{color:var(--muted);font-size:.8rem;text-transform:uppercase;letter-spacing:.04em}
.meta{border:1px solid var(--border);border-radius:8px;padding:1rem;
      background:var(--panel);margin-bottom:1.5rem}
.meta dl{display:grid;grid-template-columns:auto 1fr;gap:.35rem 1rem;margin:0}
.meta dt{color:var(--muted)}
.meta dd{margin:0;font-family:ui-monospace,SFMono-Regular,Consolas,monospace}
.tablewrap{overflow-x:auto;border:1px solid var(--border);border-radius:8px}
table{border-collapse:collapse;width:100%;min-width:640px}
th,td{text-align:left;padding:.6rem .75rem;border-bottom:1px solid var(--border);
      vertical-align:top}
th{background:var(--panel);font-weight:600;font-size:.8rem;
   text-transform:uppercase;letter-spacing:.04em;color:var(--muted)}
tr:last-child td{border-bottom:none}
.badge{display:inline-block;padding:.1rem .5rem;border-radius:999px;
       font-size:.75rem;font-weight:600;color:#fff}
code{font-family:ui-monospace,SFMono-Regular,Consolas,monospace;font-size:.85em}
.detail{color:var(--muted);font-size:.85rem;margin-top:.3rem;white-space:pre-wrap}
.verdict{font-size:1.1rem;font-weight:600;padding:.85rem 1rem;border-radius:8px;
         margin-bottom:1.5rem;border:1px solid var(--border)}
footer{color:var(--muted);font-size:.8rem;margin-top:2rem;
       border-top:1px solid var(--border);padding-top:1rem}
"""


def _badge(status: str) -> str:
    colour = _STATUS_COLOUR.get(status, "#656d76")
    return f'<span class="badge" style="background:{colour}">{html.escape(status)}</span>'


def write_html_report(
    path: str,
    results: list[TestResult],
    configuration: dict[str, Any],
) -> None:
    """Write a self-contained HTML report of a test run."""

    counts = {s: sum(1 for r in results if r.status == s)
              for s in (PASS, FAIL, SKIP, ERROR)}

    verified = {r.requirement for r in results if r.status == PASS}
    all_reqs = {r.requirement for r in results}

    if counts[FAIL] or counts[ERROR]:
        verdict, colour = "FAILED", _STATUS_COLOUR[FAIL]
    elif counts[PASS] == 0:
        # Distinguishing "nothing was verified" from "everything passed" is the
        # single most important thing this report does. A run where every test
        # skipped is not a success.
        verdict, colour = "INCONCLUSIVE - no test verified anything", _STATUS_COLOUR[SKIP]
    else:
        verdict, colour = "PASSED", _STATUS_COLOUR[PASS]

    rows = []
    for result in results:
        detail = ""
        if result.evidence:
            detail += html.escape("\n".join(result.evidence))
        if result.message:
            if detail:
                detail += "\n"
            detail += html.escape(result.message)

        rows.append(
            "<tr>"
            f"<td><code>{html.escape(result.test_id)}</code></td>"
            f"<td>{html.escape(result.name)}"
            + (f'<div class="detail">{detail}</div>' if detail else "")
            + "</td>"
            f"<td><code>{html.escape(result.requirement)}</code></td>"
            f"<td>{_badge(result.status)}</td>"
            f"<td>{result.duration_s:.2f}s</td>"
            "</tr>"
        )

    config_rows = "".join(
        f"<dt>{html.escape(str(k))}</dt><dd>{html.escape(str(v))}</dd>"
        for k, v in configuration.items() if v != ""
    )

    document = f"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>ECU Test Report</title>
<style>{_CSS}</style>
</head>
<body>
<div class="wrap">

  <h1>Automotive Body Control ECU &mdash; Test Report</h1>
  <p class="sub">System test suite, executed
     {html.escape(datetime.now().strftime('%Y-%m-%d %H:%M:%S'))}</p>

  <div class="verdict" style="border-left:4px solid {colour};color:{colour}">
    {html.escape(verdict)}
  </div>

  <div class="cards">
    <div class="card"><div class="n">{len(results)}</div><div class="l">Tests</div></div>
    <div class="card"><div class="n" style="color:{_STATUS_COLOUR[PASS]}">{counts[PASS]}</div><div class="l">Passed</div></div>
    <div class="card"><div class="n" style="color:{_STATUS_COLOUR[FAIL]}">{counts[FAIL]}</div><div class="l">Failed</div></div>
    <div class="card"><div class="n" style="color:{_STATUS_COLOUR[SKIP]}">{counts[SKIP]}</div><div class="l">Skipped</div></div>
    <div class="card"><div class="n">{len(verified)}/{len(all_reqs)}</div><div class="l">Requirements verified</div></div>
  </div>

  <div class="meta">
    <dl>{config_rows}</dl>
  </div>

  <div class="tablewrap">
  <table>
    <thead>
      <tr><th>ID</th><th>Test case</th><th>Requirement</th><th>Result</th><th>Time</th></tr>
    </thead>
    <tbody>
      {''.join(rows)}
    </tbody>
  </table>
  </div>

  <footer>
    Generated by <code>tools/can_tester/run_tests.py</code>.
    Each test is annotated with the requirement it verifies; the coverage
    figure above counts requirements demonstrated, not tests executed.
    A skipped test verifies nothing and is never counted as coverage.
  </footer>

</div>
</body>
</html>
"""

    with open(path, "w", encoding="utf-8") as handle:
        handle.write(document)
