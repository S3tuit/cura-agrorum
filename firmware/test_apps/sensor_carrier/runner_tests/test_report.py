"""Exercise the real pytest-embedded merger and our local report hooks."""
import importlib.util
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import pytest
from pytest_embedded.unity import JunitMerger, TestSuite as UnitySuite
from carrier_runner import APP


@pytest.mark.parametrize('host_outcome', [None, 'failure', 'error'])
def test_host_outcome_survives_passing_unity_preflight(tmp_path, host_outcome):
    spec = importlib.util.spec_from_file_location('carrier_report_hooks', APP/'conftest.py')
    hooks = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(hooks)
    options = dict(sensor_operation='acquire', xmlpath=str(tmp_path/'report.xml'))
    config = SimpleNamespace(getoption=options.get, option=SimpleNamespace(unity_test_report_mode='replace'))
    hooks.pytest_configure(config)
    assert config.option.unity_test_report_mode == 'merge'
    root = ET.Element('testsuites')
    suite = ET.SubElement(root, 'testsuite', tests='1', failures=str(int(host_outcome == 'failure')),
                          errors=str(int(host_outcome == 'error')), skipped='0')
    case = ET.SubElement(suite, 'testcase', name='test_sensor_carrier')
    if host_outcome:
        ET.SubElement(case, host_outcome, message='meter observation incomplete or failed').text = 'retained failure'
    ET.ElementTree(root).write(options['xmlpath'])
    serial_dir = tmp_path/'test_sensor_carrier'
    serial_dir.mkdir()
    unity = UnitySuite('test_sensor_carrier')
    unity.add_unity_test_cases('test_sensor_carrier.c:100:carrier nominal preflight:PASS\n'
                              '-----------------------\n1 Tests 0 Failures 0 Ignored\nOK\n')
    unity_path = serial_dir/'dut.xml'
    unity.dump(str(unity_path))
    session = SimpleNamespace(config=config, exitstatus=int(host_outcome is not None))
    hook = hooks.pytest_sessionfinish(session, session.exitstatus)
    next(hook)
    JunitMerger(options['xmlpath'], config.option.unity_test_report_mode).merge([str(unity_path)])
    with pytest.raises(StopIteration):
        next(hook)
    result = ET.parse(options['xmlpath'])
    cases = result.findall('.//testcase')
    assert len(cases) == 2
    assert {c.attrib['is_unity_case'] for c in cases} == {'0', '1'}
    host = next(c for c in cases if c.attrib['is_unity_case'] == '0')
    if host_outcome:
        assert host.find(host_outcome).text == 'retained failure'
    for node in result.iter():
        if node.tag in {'testsuite', 'testsuites'}:
            assert node.attrib['tests'] == '2'
            assert node.attrib['failures'] == str(int(host_outcome == 'failure'))
            assert node.attrib['errors'] == str(int(host_outcome == 'error'))
    assert session.exitstatus == int(host_outcome is not None)
