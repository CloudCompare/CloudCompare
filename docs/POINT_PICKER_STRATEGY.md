# Point Picker Enhancement: Strategy, Roadmap & Architecture

## 1. Executive summary

This document outlines how to extend this CloudCompare fork so that:

1. **Project-wide point list** (Phase 1.5): You can run Point List Picking against the **entire visible project** — no need to select one cloud first. Click any point on any visible cloud; it lands in one combined list. Same picking experience as "compare distances across any visible cloud".

2. **Pre-populated node name list** (Phase 2–3): You load a text file of point names (one per line) before you start picking. When you click a point, a small selector appears and you choose the name from the list rather than typing it out. A checklist shows which names are still unused.

3. **Source cloud column** (built into Phase 1.5): Because your cloud names are very long, each row in the list and the exported CSV will include a short **source cloud ID**. The ID is CloudCompare's built-in integer `uniqueID` — it is already shown in each cloud's Properties panel, so you can look it up at any time without guessing.

Phase 1 (export all labels) is **already implemented**. The remaining work spans Phases 1.5 → 3.

---

## 2. Current behaviour (baseline)

### 2.1 Point list picking

- **Location**: `qCC/ccPointListPickingDlg.{h,cpp}`, `qCC/ccPointPickingGenericInterface.h`, UI `qCC/ui_templates/pointListPickingDlg.ui`.
- **Trigger**: Tools → Point list picking. Requires **exactly one** selected entity.
- **Flow**: User clicks on the cloud; each click creates a **cc2DLabel** stored in a child container of that entity named `"Picked points list"`. Default label name is `"Point #N"`.
- **Export**: Only exports labels from the **current** entity's "Picked points list". There is no "export all labels in the project".

### 2.2 Labels in the scene

- **Type**: `cc2DLabel` (`libs/qCC_db`). Single-point labels have `size() == 1` and store a `PickedPoint` (entity + point index).
- **Coordinates**: `PickedPoint::getPointPosition()` returns local coordinates. Global coordinates require the parent cloud's shift/scale (`ccShiftedObject::toGlobal3d()`).

### 2.3 Entry points

- Point list picking is started from `MainWindow::activatePointListPickingMode()` (`qCC/mainwindow.cpp` ~line 7042). It enforces "exactly one entity" and passes that entity to `ccPointListPickingDlg::linkWithEntity()`.
- Export is only from within the dialog, and only for the linked entity.

---

## 3. Goals and requirements

| # | Requirement | Notes |
|---|-------------|--------|
| R1 | **One list for the whole project**: pick on any visible cloud without selecting it first. | No "exactly one entity" restriction when in project-wide mode. |
| R2 | **Source cloud ID** in list and export: short, reversible, always visible in the UI. | Use `ccHObject::getUniqueID()` — an integer already shown in Properties. |
| R3 | **Pre-populated node names**: load a text file of names before picking. | One name per line. |
| R4 | **Choose name on pick**: when clicking a point, select from the loaded list, not type. | Reduces typos; keeps names aligned with external data. |
| R5 | **Checklist / progress**: see which names are used and which remain. | In-dialog; optionally colour-coded. |
| R6 | **Export all labels** in one CSV, including source cloud ID column. | Phase 1 already exports label,x,y,z; extend to add source_id column. |

Non-goals: changing how graph software works; distance/direction/slope computation (stays in user's downstream script).

---

## 4. Source cloud ID design

### 4.1 Why not the cloud name?

Cloud names in this project are very long (e.g. full scan file paths or survey identifiers). Including the full name in the CSV would be unwieldy and error-prone.

### 4.2 Chosen approach: `uniqueID` (integer)

Every `ccHObject` in CloudCompare has a built-in integer ID assigned at creation time:

```cpp
unsigned id = entity->getUniqueID(); // e.g. 42
```

- **Already visible in the UI**: Select any cloud → Properties panel → the integer ID is shown.
- **Stable within a session**: The ID does not change while CloudCompare is open.
- **Reversible**: To find which cloud corresponds to ID 42, open Properties on each cloud and check the displayed ID. Alternatively, the exported CSV can be cross-referenced against a cloud name table (see §4.3).
- **Short**: Always a small integer (e.g. `42`), never more than ~10 characters.

### 4.3 Companion cloud name table

When exporting, write a **second CSV** alongside the main export:

```
cloud_id,cloud_name
42,My Very Long Scan File Name 2024-03-15 ...
57,Another Very Long Scan File Name ...
```

This lets you reverse-engineer any ID in post-processing without having to open CloudCompare. The file is written automatically whenever "Export all point labels" is run.

### 4.4 Example main export format

```
label_name,source_cloud_id,x,y,z
NodeA,42,1234.567,890.123,45.678
NodeB,57,1235.001,891.456,44.999
```

---

## 5. High-level architecture

### 5.1 Project-wide picking (Phase 1.5)

**Goal**: One picking session where the user does **not** select a single cloud first. They click a point; the system uses the entity under the cursor (CloudCompare's pick already returns the hit entity). A label is created and stored in a **single project-level container**; the dialog shows **one combined list** of all picks.

**Behaviour**:
- **Pick anywhere**: Picking is enabled on **any visible, pickable** point cloud or mesh. No "exactly one entity selected" requirement.
- **Best guess = hit entity**: `PickedItem::entity` already identifies which entity was clicked. The label's `PickedPoint` references that entity and index — no extra heuristics needed.
- **Single container**: One root-level `ccHObject` named `"Project picked points"` under `dbRootObject()`. Created on first use; reused for the session. All new labels go here.
- **Source column in table**: Table gains a `Source ID` column showing `entity->getUniqueID()` for each row.

**Implementation touchpoints**:

| Location | Change |
|----------|--------|
| `MainWindow::activatePointListPickingMode()` | Add "unified" path: skip one-entity check, call `linkWithRoot(dbRoot)` or pass `nullptr` + flag. |
| `ccPointListPickingDlg::linkWithRoot()` | New method. Creates/locates `"Project picked points"` under root. Sets `m_orderedLabelsContainer` to it. Clears `m_associatedEntity`. |
| `ccPointListPickingDlg::processPickedPoint()` | In unified mode: accept any `picked.entity` (not only `m_associatedEntity`). Create label from `picked.entity + picked.itemIndex`. Add to project container. |
| `ccPointListPickingDlg::updateList()` | Add `Source ID` column: `pickedPoint.entity()->getUniqueID()`. |
| Picking hub | Verify picks from any visible entity are delivered when no single entity is linked. Likely already works; confirm in testing. |

**Backward compatibility**: Existing per-entity "Picked points list" containers stay unchanged. "Export all point labels" already walks the whole tree, so both old and new containers are exported together.

### 5.2 Pre-populated node name list (Phase 2–3)

**Data**: A `QStringList m_codeList` in the dialog, plus a `QSet<QString> m_usedCodes` for checklist state.

**File format**: Plain UTF-8 text, one name per line, blank lines and `#`-comments ignored. Example:

```
# Survey nodes March 2024
NodeA
NodeB
NodeC
```

**UI additions** (in `pointListPickingDlg.ui`):
- "Load name list…" button → `QFileDialog` → parse → populate list widget.
- A list widget showing names with "Used" / "Unused" state (tick, colour, or text column).
- When a point is picked and a list is loaded: small `QDialog` (or `QComboBox` popup) pre-filled with unused names. User clicks one → label name set to that code; code marked used.
- If no list is loaded: existing behaviour (default "Point #N" name, editable in properties).

**Label name protection**: `updateList()` and auto-naming must not overwrite names that were set from the list. The existing logic already preserves non-default names for "old" labels — the same approach applies to list-assigned names.

### 5.3 Export all labels (extended)

Extend the already-implemented `MainWindow::doActionExportAllPointLabels()`:

1. Collect all `cc2DLabel` with `size() == 1` from `dbRootObject()`.
2. For each label: resolve `PickedPoint::entity()`, get `getUniqueID()` → `source_cloud_id`.
3. Resolve global coordinates (per-cloud shift/scale, already implemented).
4. Write main CSV: `label_name,source_cloud_id,x,y,z`.
5. Write companion CSV (same path, suffix `_clouds`): `cloud_id,cloud_name` for every cloud that appears in the export.

---

## 6. Roadmap (phased)

### Phase 1 — Export all point labels ✅ COMPLETE

Menu action already implemented. Exports `label,x,y,z` for every single-point 2D label in the project.

**Remaining**: Add `source_cloud_id` column and companion cloud-name table (small extension, can be Phase 1b).

---

### Phase 1b — Add source cloud ID to export

**Files**: `qCC/mainwindow.cpp` (`doActionExportAllPointLabels`).

**Steps**:
1. In the export loop, add `pickedPoint.entity()->getUniqueID()` as a column after `label_name`.
2. Accumulate a `QMap<unsigned, QString>` of `id → entity->getName()` for every entity seen.
3. After writing the main CSV, write `<basename>_clouds.csv` with that map.

**Deliverable**: Export now outputs `label,source_id,x,y,z` and a companion `_clouds.csv`.

---

### Phase 1.5 — Unified project-wide picking

**Files**: `qCC/mainwindow.{h,cpp}`, `qCC/ccPointListPickingDlg.{h,cpp}`, `qCC/ui_templates/pointListPickingDlg.ui`.

**Steps**:
1. Add a new menu action "Point list picking (all clouds)" (or add a "Pick from all clouds" checkbox to the dialog).
2. In `MainWindow`: new activation path that does not check for a single selection; calls `m_plpDlg->linkWithRoot(dbRootObject())`.
3. In `ccPointListPickingDlg`:
   - Add `linkWithRoot(ccHObject* root)`: finds or creates `"Project picked points"` container under root; sets it as `m_orderedLabelsContainer`; sets `m_associatedEntity = nullptr`.
   - Modify `processPickedPoint()`: when `m_associatedEntity == nullptr`, use `picked.entity` directly for the label's `PickedPoint`; add label to the project container.
   - In `updateList()`: add `Source ID` column populated from `label->getPickedPoint(0).entity()->getUniqueID()`.
4. Verify picking hub delivers events from any visible entity when no single entity is linked.

**Deliverable**: User opens "Point list picking (all clouds)", clicks on any cloud, sees one combined table with Source ID column.

---

### Phase 2 — Code/name list load and display

**Files**: `qCC/ccPointListPickingDlg.{h,cpp}`, `qCC/ui_templates/pointListPickingDlg.ui`.

**Steps**:
1. Define and document file format (plain text, one name per line).
2. Add "Load name list…" button and `QListWidget` (or table) to dialog UI.
3. Add `QStringList m_codeList` and `QSet<QString> m_usedCodes` to dialog class.
4. Parse file on load; populate list widget; show "Unused" for all entries.
5. Persist last-used file path in `QSettings`.

**Deliverable**: User can load a name list and see all names in the dialog.

---

### Phase 3 — Pick from list and checklist

**Files**: `qCC/ccPointListPickingDlg.{h,cpp}`.

**Steps**:
1. In `processPickedPoint()`: if `m_codeList` is non-empty, open a small name-selector dialog (filterable `QComboBox` or `QListWidget` with search). On confirm, call `label->setName(selectedCode)`; add `selectedCode` to `m_usedCodes`.
2. Ensure auto-naming in `updateList()` skips labels whose names are in `m_codeList` (treat like existing custom names).
3. Update list widget: mark selected code as "Used" (tick, strikethrough, or colour).
4. Optional: highlight "next unused" name to guide sequential picking.

**Deliverable**: Full "load list → pick point → choose name → see progress" workflow.

---

### Phase 4 — Polish

- Export format options (local vs global coords, delimiter choice).
- Keyboard shortcut for "Export all point labels".
- Tooltips and translation hooks.
- Support CSV (first column = name) in code list import.

---

## 7. File and class changes (summary)

| Area | File(s) | Change |
|------|--------|--------|
| Export source ID (Phase 1b) | `qCC/mainwindow.cpp` | Add `source_cloud_id` column; write companion `_clouds.csv`. |
| Unified picking (Phase 1.5) | `qCC/mainwindow.{h,cpp}` | New activation path without one-entity requirement. |
| Unified picking | `qCC/ccPointListPickingDlg.{h,cpp}` | `linkWithRoot()`; unified mode in `processPickedPoint()`; Source ID column in `updateList()`. |
| Unified picking | `qCC/ui_templates/pointListPickingDlg.ui` | Add Source ID column to table widget. |
| Name list (Phase 2) | `qCC/ccPointListPickingDlg.{h,cpp}` | `m_codeList`, `m_usedCodes`; "Load name list" button; list widget. |
| Name list | `qCC/ui_templates/pointListPickingDlg.ui` | New button and list widget. |
| Pick from list (Phase 3) | `qCC/ccPointListPickingDlg.{h,cpp}` | Name-selector dialog on pick; checklist update. |

---

## 8. Risks and mitigations

| Risk | Mitigation |
|------|------------|
| `uniqueID` changes if project is saved and reloaded | IDs are re-assigned on load. The companion `_clouds.csv` is always written alongside the main export so the mapping stays with the data. Document this in the UI tooltip. |
| Global coordinates differ per cloud (shift/scale) | Reuse the same resolution as the existing export: `ccShiftedObject::toGlobal3d()` per label's entity. |
| Picking hub only delivers clicks for the linked entity | Test first. If needed, remove the entity filter in `onItemPicked()` when in unified mode, or register with `exclusive=false` and `autoStartPicking` on all visible clouds. |
| User picks a point but cancels the name selector | Don't create a label on cancel. Or create with default name and let user rename in properties. Define desired behaviour in Phase 3. |
| Name list has duplicates | Allow duplicates; warn on load. Track usage per position, not per string, if the same name can be used more than once. |

---

## 9. How to build and use the updated software

### 9.1 Overview for a first-time OS contributor

CloudCompare is a C++ / Qt application built with CMake. You don't need to understand all of it — just the files you'll change. The workflow is:

1. Edit the C++ source files listed in §7.
2. Run CMake + your compiler to produce a new `CloudCompare.exe`.
3. Run that EXE instead of the released version.
4. Test your changes interactively.

### 9.2 Prerequisites (Windows)

| Tool | Where to get it |
|------|----------------|
| **CMake** ≥ 3.20 | cmake.org |
| **Visual Studio 2022** (Community is free) | visualstudio.microsoft.com — install "Desktop development with C++" workload |
| **Qt 5.15** (or Qt 6.x if already set up) | qt.io → Qt Online Installer → select MSVC 2019 64-bit |
| **Git** | already installed (you're using this repo) |

### 9.3 First build

```bash
# 1. From the repo root, create a build directory
mkdir build && cd build

# 2. Configure (adjust Qt path to match your installation)
cmake .. -G "Visual Studio 17 2022" -A x64 \
  -DCMAKE_PREFIX_PATH="C:/Qt/5.15.2/msvc2019_64"

# 3. Build (Release is faster; Debug gives better error messages while developing)
cmake --build . --config Release --parallel 4
```

The first build takes 10–20 minutes. Subsequent builds (after editing a few files) take under a minute because CMake only recompiles changed files.

The output EXE will be at something like `build/qCC/Release/CloudCompare.exe`.

### 9.4 Iterative development workflow

After the first successful build, your loop is:

1. Edit a `.cpp` or `.h` file in `qCC/` or `libs/qCC_db/`.
2. If you edited a `.ui` file (Qt Designer XML), CMake will regenerate the C++ header automatically on the next build.
3. Run `cmake --build . --config Release --parallel 4` again from the `build/` directory.
4. Launch `build/qCC/Release/CloudCompare.exe`.
5. Test your change, then go back to step 1.

You do **not** need to re-run the `cmake ..` configuration step unless you add new files or change `CMakeLists.txt`.

### 9.5 How to use the new features (end-user workflow)

Once the updated software is built, your daily workflow becomes:

#### Project-wide point list picking

1. Open CloudCompare, load all your PLY scan files as usual (no need to join them into one cloud).
2. Go to **Tools → Point list picking (all clouds)** (new menu item).
3. The Point List Picking dialog opens. The table is empty; there is now a **Source ID** column.
4. Click any point on any visible cloud. A marker appears; the point lands in the table with its coordinates and the **Source ID** of the cloud you clicked.
5. Continue clicking across different clouds. All picks accumulate in one table.
6. When done, click **Accept**. All labels are stored in the `"Project picked points"` container (visible in the DB tree on the left).
7. Go to **File → Batch export → Export all point labels**. Save as CSV. You get:
   - `myfile.csv`: `label_name,source_cloud_id,x,y,z`
   - `myfile_clouds.csv`: `cloud_id,cloud_name` mapping

To look up which cloud a `source_cloud_id` belongs to: select any cloud → **Properties panel** (bottom-left) → find the `ID` field. It matches the integer in your CSV.

#### Pre-populated node names (Phase 2–3, once implemented)

1. Before picking, prepare a plain text file (`nodes.txt`) with one name per line:
   ```
   NodeA
   NodeB
   NodeC
   ```
2. In the Point List Picking dialog, click **Load name list…** and select your file.
3. The name list widget populates with all names, all showing "Unused".
4. Click a point on any cloud. A small name selector appears listing your names.
5. Click the correct name → the label is created with that name; the name is ticked off in the list.
6. Repeat until all names are assigned (or as many as you need).
7. Export as above — `label_name` column now contains your node names.

---

## 10. References (in-repo)

- Point list picking: `qCC/ccPointListPickingDlg.cpp` (`linkWithEntity`, `processPickedPoint`, `exportToASCII`, `updateList`).
- Picking hub / listener: `qCC/ccPointPickingGenericInterface.h`, `libs/CCPluginAPI/include/ccPickingHub.h`.
- Labels: `libs/qCC_db/include/cc2DLabel.h` (`PickedPoint`, `getPointPosition`, `getName`/`setName`).
- Global label collection: `qCC/mainwindow.cpp` `doActionExportAllPointLabels` (lines ~9338–9427).
- Main window entry points: `qCC/mainwindow.cpp` `activatePointListPickingMode` (~line 7042).
- Unique ID: `ccHObject::getUniqueID()` in `libs/qCC_db/include/ccHObject.h`.
