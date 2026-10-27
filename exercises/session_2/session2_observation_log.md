# Session 2 observation log
Use this file to keep evidence while working. You may also keep the same information in your own notes.

## Reference model joint table
| Joint | Type | Parent link | Child link | Origin `xyz` | Axis |
| --- | --- | --- | --- | --- | --- |
| `world_to_base` | | | | | |
| `joint_1` | | | | | |
| `joint_2` | | | | | |
| `tool_joint` | | | | | |

## Reference model screenshots
| Screenshot | File name or location | What it shows |
| --- | --- | --- |
| Zero configuration | | |
| Nonzero configuration | | |

## Manual TF prediction
With both sliders at zero, predicted `world -> tool0` position:

```text
x:
y:
z:
```

Measured with `tf2_echo world tool0`:

```text
x:
y:
z:
```

## My model validation
| Check | Result or pasted output |
| --- | --- |
| `xmllint --noout my_robot.urdf` | |
| `check_urdf my_robot.urdf` | |
| `/joint_states` contains `joint_1` and `joint_2` | |
| `tf2_echo world tool0` works | |
| `frames.pdf` generated | |

## Diagnosis challenge
| Fault | XML detects? | URDF detects? | RViz detects? | TF tool detects? | Short explanation |
| --- | --- | --- | --- | --- | --- |
| Misspelled child link | | | | | |
| Wrong RViz Fixed Frame | | | | | |
| Wrong visual origin only | | | | | |
| Wrong joint origin only | | | | | |

## Xacro variant
Changed property:

```text

```

Observed effect in RViz and TF:

```text

```
