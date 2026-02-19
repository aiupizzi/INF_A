from geojson_exporter.exporter import transform_defects


def test_transform_defects_generates_required_properties():
    defects = [
        {
            'defect_id': 'D-001',
            'severity': 'high',
            'confidence': 0.92,
            'image_path': '/tmp/image.jpg',
            'suggested_action': 'inspect immediately',
            'local_coordinates': {'x': 10.0, 'y': 2.0, 'z': 1.2},
        }
    ]
    mapping = {
        'target_crs': {'name': 'test_crs'},
        'transform': {
            'affine_2d': {
                'scale_x': 2.0,
                'scale_y': 1.0,
                'rotation_deg': 0.0,
                'translation_x': 1.0,
                'translation_y': -1.0,
            }
        },
    }

    output = transform_defects(defects, mapping)
    feature = output['features'][0]

    assert output['type'] == 'FeatureCollection'
    assert feature['properties']['defect_id'] == 'D-001'
    assert feature['properties']['suggested_action'] == 'inspect immediately'
    assert feature['geometry']['coordinates'] == [21.0, 1.0, 1.2]
