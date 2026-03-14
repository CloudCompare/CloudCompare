# Point Picker Enhancement: Strategy, Roadmap & Architecture

## 1. Executive summary

This document outlines how to extend this CloudCompare fork so that:

1. **Code list workflow**: You can load a list of point codes (from your graph software), then when picking points on the cloud you **choose a code from that list** instead of typing. The UI helps you work through the list so you can see what’s done and avoid missing or misspelling codes.

2. **Project-wide label export**: You can **export all point labels** from the current project in one go (one file), instead of exporting per cloud/segment.

The design reuses the existing Point List Picking and label infrastructure where possible, and adds a small number of new entry points and data flows.

---

## 2. Current behaviour (baseline)

### 2.1 Point list picking

- **Location**: `qCC/ccPointListPickingDlg.{h,cpp}`, `qCC/ccPointPickingGenericInterface.h`, UI `qCC/ui_templates/pointListPickingDlg.ui`.
- **Trigger**: Tools → Point list picking (or equivalent). Requires **exactly one** selected entity (one point cloud or one mesh).
- **Flow**: User clicks on the cloud; each click creates a **cc2DLabel** with one picked point. Labels are stored in a child container of that entity named `"Picked points list"` (see `s_pickedPointContainerName`). Default label name is `"Point #" + (startIndex + i)`.
- **Naming**: In `updateList()`, any label that already has a custom name (not the default `"Point #…"`) is left unchanged; new labels get the default. So custom names are possible (e.g. via properties), but there is no in-dialog way to pick from a predefined list.
- **Export**: Export (ASCII xyz / ixyz / gxyz / **lxyz** = label name, x, y, z) only exports labels from the **current** entity’s “Picked points list”. There is no “export all labels in the project”.

### 2.2 Labels in the scene

- **Type**: `cc2DLabel` (libs/qCC_db, `cc2DLabel.{h,cpp}`). Single-point labels have `size() == 1` and store a `PickedPoint` (cloud/mesh + index + optional UV for meshes).
- **Placement**: Labels are children of a hierarchy object (typically under a specific cloud’s “Picked points list”). The DB root is available via `MainWindow::TheInstance()->dbRootObject()` (returns `ccHObject*`). All 2D labels in the project can be collected with:
  - `root->filterChildren(container, true, CC_TYPES::LABEL_2D)` (recursive).
- **Coordinates**: `PickedPoint::getPointPosition()` returns local coordinates. Global coordinates (e.g. lat/long/depth) require the parent cloud’s shift/scale (e.g. `ccShiftedObject` / `toGlobal3d()`), which is already used in the existing export and in the “show global coordinates” option.

### 2.3 Entry points

- Point list picking is started from `MainWindow::activatePointListPickingMode()` in `qCC/mainwindow.cpp` (around 7042). It enforces “exactly one entity” and passes that entity to `ccPointListPickingDlg::linkWithEntity()`.
- Export is only from within the dialog, and only for `m_orderedLabelsContainer` of the linked entity.

---

## 3. Goals and requirements

| # | Requirement | Notes |
|---|-------------|--------|
| R1 | Load a **list of point codes** (from graph software). | Format TBD: e.g. one code per line, or CSV (code, optional lat, long, depth). |
| R2 | When picking a point on the cloud, **assign a label by selecting a code from that list** (no typing). | Reduces typos and keeps labels aligned with the graph. |
| R3 | **Checklist / progress** over the code list so the user can see which codes are already assigned and ensure none are missed or misspelled. | e.g. list view with “used”/“unused” or checkmarks. |
| R4 | **Export all point labels** in the project **in one go** to a single file. | Not per cloud/segment; one export for the whole scene. |
| R5 | Export format compatible with downstream script (e.g. label + lat, long, depth in metres). | Same or similar to current lxyz-style export, with global coordinates where needed. |

Non-goals for this roadmap: changing how graph software works, or implementing distance/direction/slope (those stay in the user’s existing script).

---

## 4. High-level architecture

### 4.1 Code list and “pick from list” workflow

- **Data**: A **code list** (ordered set of strings) loaded from file and kept in memory (e.g. in the point list picking dialog or a small helper).
- **UI**: In the Point List Picking dialog (or an extended version):
  - **Load list**: Button/menu to load a file → parse into list of codes.
  - **List panel**: Show the list of codes with a “used” state (e.g. which code was assigned to which pick, or simply “used”/“unused”). Optionally allow reordering or filtering.
  - **On pick**: When the user clicks a point, instead of (or in addition to) creating a label with a default name:
    - Open a small chooser (e.g. combo box or list widget) pre-filled with the loaded codes.
    - User selects one code → create (or update) the label with that name; mark that code as used.
  - **Persistence**: Code list can be session-only; optionally remember last path for “load list”.
- **Label naming**: Use the selected code as the label’s name (`cc2DLabel` inherits `ccHObject::setName()`). Ensure the dialog’s `updateList()` (or equivalent) does **not** overwrite user-chosen names with `"Point #…"` when a code was selected. The existing logic already preserves non-default names for “old” labels; the same idea applies to “code-assigned” names.

### 4.2 Project-wide label export

- **Entry point**: New menu action, e.g. **File → Export → Export all point labels** (or under a dedicated “Labels” submenu). No need to have the Point List Picking dialog open; works on the current DB state.
- **Logic**:
  1. Get root: `MainWindow::TheInstance()->dbRootObject()`.
  2. Collect all 2D labels: `root->filterChildren(labels, true, CC_TYPES::LABEL_2D)`.
  3. Filter to **point labels** only: keep only labels with `size() == 1` (single-point labels).
  4. For each label: get `getPickedPoint(0)`, then `getPointPosition()` and the owning entity’s global shift/scale to compute global coordinates (lat, long, depth in metres if that’s the convention).
  5. Write one file (e.g. CSV or tab-separated): at least `label_name, x, y, z` (or `label_name, lat, long, depth`); format chosen to match the user’s downstream script.
- **File format**: Prefer configurable (e.g. dropdown: “label,x,y,z” vs “label,lat,long,depth”) and consistent with existing ASCII export (e.g. `AsciiFilter`-style or same columns as lxyz). Reuse existing “apply global shift/scale?” behaviour where applicable.

### 4.3 Optional: multi-entity picking

- **Current**: Picking is tied to one entity; labels are stored under that entity.
- **Possible extension**: Allow “pick on any visible cloud” and still store labels in a **single project-level container** (e.g. a dedicated “Project point labels” group under the root). That would make “export all” naturally include every pick from every cloud. This is a larger change (selection model, which entity to attach to, how to handle shift/scale per label).
- **Recommendation**: Phase 1 can keep **single-entity** picking and only add **project-wide export** (so the user can still have multiple clouds, each with its own “Picked points list”, but export all of them in one go). If needed, Phase 2 can introduce a “unified label container” and/or multi-entity picking.

---

## 5. Component design

### 5.1 Code list

- **Class**: Either a simple `QStringList` (or `std::vector<QString>`) in the dialog, or a small class e.g. `PointCodeList` that holds:
  - `QStringList codes`;
  - optional `QSet<QString> usedCodes` (or a map code → cc2DLabel* for “which label has this code”).
- **File format**: Keep it simple in v1, e.g.:
  - One code per line (trimmed, skip empty).
  - Or CSV: first column = code, optional columns = lat, long, depth (for future validation/display only).
- **Storage**: Member of `ccPointListPickingDlg` (or of a derived class if we split “basic” vs “code list” mode).

### 5.2 UI changes (Point List Picking dialog)

- **New widgets** (in `pointListPickingDlg.ui` or a separate small dialog):
  - “Load code list” button → file dialog → load and parse.
  - Optional: “Clear code list”.
  - A list/table showing codes and “Used”/“Unused” (and optionally which label name they have). This can be a second panel or a collapsible section.
  - When a point is picked: if code list is loaded, show a **code selector** (e.g. `QComboBox` with the list, or a `QListWidget` with search); on “OK” create the label with the selected code as name and mark that code as used. If no code list is loaded, keep current behaviour (default “Point #N”).
- **Table**: The existing table (Index, X, Y, Z) can stay; optionally add a “Label name” column that shows the code (already reflected by `labels[i]->getName()` in export). The “start index” and “marker size” behaviour can remain.

### 5.3 Export all labels

- **New function**: e.g. `MainWindow::exportAllPointLabels()`.
  - Collect all `cc2DLabel` with `size() == 1` from `dbRootObject()`.
  - Resolve global coordinates using each label’s `PickedPoint::entity()` and its shift/scale (same as in `ccPointListPickingDlg::exportToASCII` for lxyz).
  - Ask user for file path and optional format (label,x,y,z vs label,lat,long,depth if different).
  - Write one file.
- **Menu**: Register in `mainwindow.cpp` and in the UI (e.g. `mainWindow.ui` or a menu bar action). Enable when the DB is not empty (or always); disable when there are no point labels if desired.

### 5.4 Data flow (code list + export)

```
[Graph software] → export list of codes (e.g. CSV or TXT)
       ↓
[CloudCompare]  → File or dialog: "Load code list" → parse → store in dialog
       ↓
User picks point → dialog shows code selector → user picks code → create cc2DLabel with setName(code)
       ↓
Checklist shows code as "used"; user can continue until list is done
       ↓
File → Export all point labels → collect all cc2DLabel (size==1) from root → write one file (label, x, y, z or lat, long, depth)
       ↓
[Downstream script] → reads export, computes distance/direction/slope for pairs that have a connecting line
```

---

## 6. Roadmap (phased)

### Phase 1 – Foundation (minimal slice)

- **1.1** Add **Export all point labels**:
  - Implement `MainWindow::exportAllPointLabels()` (or equivalent) that:
    - Uses `dbRootObject()->filterChildren(..., true, CC_TYPES::LABEL_2D)`.
    - Keeps only single-point labels.
    - Resolves global coordinates (shift/scale) and writes one ASCII file (e.g. label,x,y,z).
  - Add menu action and wire it in the main window.
- **1.2** Tests: Load a project with labels under two different clouds; run “Export all point labels”; confirm one file with all of them and correct coordinates.

**Deliverable**: One menu action to export every point label in the project to a single file.

### Phase 2 – Code list load and display

- **2.1** Define code list file format (e.g. one code per line, UTF-8).
- **2.2** In Point List Picking dialog: add “Load code list” and a simple list widget showing loaded codes (no “used” state yet).
- **2.3** Persist “last used path” for the code list file (e.g. QSettings).

**Deliverable**: User can load a code list and see it in the dialog.

### Phase 3 – Pick from list and checklist

- **3.1** On point pick: if a code list is loaded, show a selector (combo or list) with the loaded codes; user selects one → create label with that name (`setName(selectedCode)`).
- **3.2** Ensure `updateList()` (and any auto-naming) does not overwrite these names (treat “code-assigned” names like existing custom names).
- **3.3** Track which codes have been assigned (e.g. set or map). Update the code list widget to show “Used”/“Unused” (e.g. checkmark or colour).
- **3.4** Optional: “Next suggested code” (e.g. first unused in list) to speed up sequential picking.

**Deliverable**: Full “load list → pick point → choose code → see progress” workflow.

### Phase 4 – Polish and format options

- **4.1** Export all: add format options (e.g. “Local x,y,z” vs “Global lat, long, depth”) and possibly delimiter (comma vs tab).
- **4.2** Code list: support CSV (first column = code) and “one per line”; optional columns for lat/long/depth for display only.
- **4.3** Small UX improvements: keyboard shortcut for “Export all point labels”, tooltips, and any needed translation hooks.

**Deliverable**: Robust, configurable export and flexible code list import.

---

## 7. File and class changes (summary)

| Area | File(s) | Change |
|------|--------|--------|
| Export all | `qCC/mainwindow.{h,cpp}` | New method `exportAllPointLabels()`; new menu action. |
| Export all | `qCC/ui_templates/mainWindow.ui` (or menu bar) | Add “Export all point labels” action. |
| Code list | `qCC/ccPointListPickingDlg.{h,cpp}` | Members for code list + “used” set; “Load code list” and code selector on pick. |
| Code list | `qCC/ui_templates/pointListPickingDlg.ui` | Widgets: Load list button, code list widget, optional “used” column. |
| Shared | `libs/qCC_io/` or `qCC/` | Reuse existing ASCII/export helpers for global coords and file writing where possible. |

New classes are only needed if we want a separate “PointCodeList” or a dedicated “Export all labels” dialog; otherwise, logic can live in `MainWindow` and `ccPointListPickingDlg`.

---

## 8. Risks and mitigations

| Risk | Mitigation |
|------|------------|
| Global coordinates differ per cloud (shift/scale) | Reuse the same resolution as in `ccPointListPickingDlg::exportToASCII` (per-label entity and its shift/scale). |
| Many labels → slow or large export | Single pass over `filterChildren` is O(n); file I/O is sequential. For very large projects, consider progress bar. |
| Code list duplicates (same code twice) | Allow duplicates in the list but track “used” per occurrence or show a warning when loading. |
| User picks point but cancels code selector | Don’t create a label; or create with default name and let user rename. Define desired behaviour in Phase 3. |

---

## 9. Success criteria

- User can load a list of point codes from a file and see them in the Point List Picking dialog.
- User can pick a point and assign a label by selecting a code from that list (no typing).
- User can see which codes have been assigned (checklist) and export all point labels in the project to one file with correct global coordinates.
- Downstream script can consume the exported file (e.g. label, lat, long, depth) and compute distance/direction/slope for connected pairs as it does today.

---

## 10. References (in-repo)

- Point list picking: `qCC/ccPointListPickingDlg.cpp` (e.g. `linkWithEntity`, `processPickedPoint`, `exportToASCII`, `updateList`).
- Picking hub / listener: `qCC/ccPointPickingGenericInterface.h`, `ccPickingListener`.
- Labels: `libs/qCC_db/include/cc2DLabel.h` (`PickedPoint`, `getPointPosition`, `getName`/`setName`).
- Global label collection: `qCC/ccGraphicalSegmentationTool.cpp` (e.g. around 1544–1547) uses `dbRootObject()->filterChildren(..., true, CC_TYPES::LABEL_2D)`.
- Main window and DB: `qCC/mainwindow.{h,cpp}`, `qCC/db_tree/ccDBRoot.h` (`getRootEntity()`).
