# Copyright 2018 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import glob
from pathlib import Path

from lxml import etree

from sros2.policy import (
    get_policy_schema,
    get_transport_schema,
    get_transport_template,
)

# Get paths
policy_xsd_path = get_policy_schema('policy.xsd')
permissions_xsl_path = get_transport_template('dds', 'permissions.xsl')
permissions_xsd_path = get_transport_schema('dds', 'permissions.xsd')

# Parse files
policy_xsd = etree.XMLSchema(etree.parse(policy_xsd_path))
permissions_xsl = etree.XSLT(etree.parse(permissions_xsl_path))
permissions_xsd = etree.XMLSchema(etree.parse(permissions_xsd_path))

for policy_xml_path in glob.glob('*.policy.xml'):

    # Get policy
    policy_xml = etree.parse(policy_xml_path)
    policy_xml.xinclude()

    # Validate policy schema
    policy_xsd.assertValid(policy_xml)

    # Transform policy
    permissions_xml = permissions_xsl(policy_xml)

    # Validate permissions schema
    permissions_xsd.assertValid(permissions_xml)

    # Get permissions directory
    policy_name = Path(policy_xml_path).name
    index_of_dot = policy_name.index('.')
    policy_name = policy_name[:index_of_dot]
    permissions_dir = Path('permissions') / policy_name
    permissions_dir.mkdir(parents=True, exist_ok=True)

    # Output permissions
    permissions_xml_path = permissions_dir / 'permissions.xml'
    with open(permissions_xml_path, 'w') as f:
        f.write(etree.tostring(permissions_xml, pretty_print=True).decode())
