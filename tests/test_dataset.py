from pathlib import Path

import pytest
import yaml

from mix_dataset import examples


def real_sample(root: Path, annotation: str):
    (root/'train/images').mkdir(parents=True)
    (root/'train/labels').mkdir(parents=True)
    (root/'data.yaml').write_text(yaml.safe_dump({'names':['Orange_hammer','bottle']}))
    (root/'train/images/sample.jpg').touch()
    (root/'train/labels/sample.txt').write_text(annotation)


def test_roboflow_polygon_becomes_detection_box_with_remapped_class(tmp_path):
    real_sample(tmp_path, '0 0.1 0.2 0.5 0.2 0.3 0.8\n1 0.4 0.5 0.2 0.3\n')
    _, rows = next(examples(tmp_path, 'real', 'train'))
    assert rows[0].split()[0] == '1'
    assert list(map(float, rows[0].split()[1:])) == pytest.approx([0.3, 0.5, 0.4, 0.6])
    assert rows[1].split()[0] == '0'
    assert list(map(float, rows[1].split()[1:])) == pytest.approx([0.4, 0.5, 0.2, 0.3])


@pytest.mark.parametrize('coordinates', ['0.1 0.2 0.3', '0.1 0.2 0.3 0.4 0.5',
                                           '0.1 0.2 0.1 0.2 0.1 0.2', 'nan 0.5 0.2 0.3'])
def test_malformed_or_empty_boxes_are_rejected(tmp_path, coordinates):
    real_sample(tmp_path, '0 '+coordinates)
    with pytest.raises(ValueError, match='bad YOLO annotations'):
        list(examples(tmp_path, 'real', 'train'))
