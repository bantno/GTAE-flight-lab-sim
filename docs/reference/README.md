# Reference documents

Drop the FDC / Beaver source documents here. They are **gitignored** — this
repo is public and these are someone else's copyright, so we keep them locally
and cite rather than redistribute.

## What we need them for

`docs/MODEL_NOTES.md` establishes that the inertia matrix in `AE4803Sim.slx` is
asymmetric, which no sign convention can explain and which the 6DOF block does
not silently repair. What is **not** established is the correct *sign* of the
symmetric product of inertia — and that uncertainty (1.97 % of signal scale) is
larger than the asymmetry it would fix (1.09 %).

These are the primary sources for where `Jxz = 117.64 kg·m²` came from and
under which convention.

## Files to fetch

From the FDC toolbox project (Marc O. Rauw, Delft University of Technology):

| file | what it is | link |
|---|---|---|
| `FDC12-report.pdf` | FDC 1.2 user manual, 2.0 MB — **most likely to carry the Beaver data table** | [download](https://sourceforge.net/projects/dutchroll/files/FDC%20Toolbox%20-%20documentation/FDC%20user%20manual/FDC12-report.pdf/download) |
| `thesis_mrauw_part1.pdf` | MSc thesis Part 1, "Implementation of a model library in Simulink", 5.2 MB | [download](https://sourceforge.net/projects/dutchroll/files/FDC%20Toolbox%20-%20documentation/MSc%20thesis/thesis_mrauw_part1.pdf/download) |
| `thesis_mrauw_part2.pdf` | MSc thesis Part 2, Beaver autopilot analysis, 3.8 MB | [download](https://sourceforge.net/projects/dutchroll/files/FDC%20Toolbox%20-%20documentation/MSc%20thesis/thesis_mrauw_part2.pdf/download) |

Folder listing:
<https://sourceforge.net/projects/dutchroll/files/FDC%20Toolbox%20-%20documentation/>
Project home: <https://dutchroll.sourceforge.net/>

**These must be fetched with a browser.** SourceForge serves a signed
meta-refresh interstitial and then puts a Cloudflare bot challenge in front of
scripted clients, so curl/urllib get HTML or a 403 rather than the PDF.

## What to look for once they are here

1. The Beaver's mass and inertia table — expect `Ix`, `Iy`, `Iz`, `Jxz` with
   values near 5368.39 / 6928.93 / 11158.75 / 117.64 kg·m². (Note the lab's
   `Ixx` and `Izz` are each exactly 419.579 larger than those, which is a
   deliberate change by someone, not a typo.)
2. **The sign convention**: whether `Jxz` is defined as `∫xz dm` or as its
   negative, and therefore whether the tensor off-diagonal is `−117.64` or
   `+117.64`. That single sign is the open question.
3. The moment equations as printed, which pin the convention down regardless of
   how the symbol is defined in prose.

Then set `inertia_mode` in `config/sim.toml` to `symmetric_neg` or
`symmetric_pos` accordingly, and update `docs/MODEL_NOTES.md` §2.1.
