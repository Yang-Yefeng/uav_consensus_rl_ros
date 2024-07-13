import xml.etree.ElementTree as ET


def XML_Create(filename: str, rootname: str, rootmsg: dict, is_pretty: bool=False):
    """
    :brief:                 创建一个XML文档
    :param filename:        文件名
    :param rootname:        根节点名字
    :param rootmsg:         根节点信息
    :param is_pretty:       是否进行XML美化
    :return:                None
    """
    new_xml = ET.Element(rootname, attrib=rootmsg)  # 最外面的标签名
    et = ET.ElementTree(new_xml)
    et.write(filename,
             encoding='utf-8',
             xml_declaration=True)
    if is_pretty:
        XML_Pretty_All(filename)


def XML_Load(filename: str) -> ET.Element:
    """
    :brief:             加载一个XML文件
    :param filename:    文件名
    :return:            根节点
    """
    xml_root = ET.parse(filename).getroot()
    return xml_root


def XML_FindNode(nodename: str, root: ET.Element) -> ET.Element:
    """
    :brief:             寻找XML文件中的节点
    :param nodename:    节点名
    :param root:        根节点
    :return:            该节点
    """
    for child in root:
        # print(child.tag)
        if child.tag == nodename:
            return child
    print('No node named' + nodename + 'here...')


def XML_GetTagValue(node: ET.Element)->dict:
    """
    :brief:         得到某一个节点的标签
    :param node:    该节点
    :return:        标签信息
    """
    nodemsg = {}
    for item in node:
        nodemsg[item.tag] = item.text
    return nodemsg


def XML_InsertNode(filename: str, nodename: str, nodemsg: dict, is_pretty: bool=False):
    """
    :brief:                 在XML文档中插入结点
    :param filename:        文件名
    :param nodename:        新的节点名
    :param nodemsg:         新的节点的信息
    :param is_pretty:       是否进行XML美化
    :return:                None
    """
    xml_root = ET.parse(filename).getroot()
    new_node = ET.SubElement(xml_root, nodename)
    for key, value in nodemsg.items():
        _node = ET.SubElement(new_node, key)
        _node.text = str(value)
    et = ET.ElementTree(xml_root)
    et.write(filename,
             encoding='utf-8',
             xml_declaration=True)
    if is_pretty:
        XML_Pretty_All(filename)


def XML_InsertMsg2Node(filename: str, nodename: str, msg: dict, is_pretty: bool=False):
    """
    :brief:                 在某一个节点中插入信息
    :param filename:        文件名
    :param nodename:        节点名
    :param msg:             信息
    :param is_pretty:       是否进行XML美化
    :return:                None
    """
    xml = ET.parse(filename)
    xml_root = xml.getroot()
    for child in xml_root:
        if child.tag == nodename:
            for key, value in msg.items():
                _node = ET.SubElement(child, key)
                _node.text = str(value)
            # break
    et = ET.ElementTree(xml_root)
    et.write(filename,
             encoding='utf-8',
             xml_declaration=True)
    if is_pretty:
        XML_Pretty_All(filename)


def XML_RemoveNode(filename: str, nodename: str, is_pretty=False):
    """
    :brief:                 从XML文档中移除某节点
    :param filename:        文件名
    :param nodename:        节点名
    :param is_pretty:       是否进行XML美化
    :return:                None
    """
    xml = ET.parse(filename)
    xml_root = xml.getroot()
    for child in xml_root:
        if child.tag == nodename:
            xml_root.remove(child)
            # break
    et = ET.ElementTree(xml_root)
    et.write(filename,
             encoding='utf-8',
             xml_declaration=True)
    if is_pretty:
        XML_Pretty_All(filename)


def XML_RemoveNodeMsg(filename: str, nodename: str, msgname: str, is_pretty=False):
    """
    :brief:             从某一个节点中移除某些信息
    :param filename:    文件名
    :param nodename:    节点名
    :param msgname:     信息的名字
    :param is_pretty:   是否进行XML美化
    :return:            None
    """
    xml = ET.parse(filename)
    xml_root = xml.getroot()
    for child in xml_root:
        if child.tag == nodename:
            for msg in child:
                if msg.tag == msgname:
                    child.remove(msg)
    et = ET.ElementTree(xml_root)
    et.write(filename,
             encoding='utf-8',
             xml_declaration=True)
    if is_pretty:
        XML_Pretty_All(filename)


def XML_Pretty(element: ET.Element, indent: str='\t', newline: str='\n', level: int=0):
    """
    :brief:             XML美化
    :param element:     节点元素
    :param indent:      缩进
    :param newline:     换行
    :param level:       缩进个数
    :return:            None
    """
    if element:
        if (element.text is None) or element.text.isspace():  # 如果element的text没有内容
            element.text = newline + indent * (level + 1)
        else:
            element.text = newline + indent * (level + 1) + element.text.strip() + newline + indent * (level + 1)
            # else:  # 此处两行如果把注释去掉，Element的text也会另起一行
            # element.text = newline + indent * (level + 1) + element.text.strip() + newline + indent * level
    temp = list(element)
    for subelement in temp:
        if temp.index(subelement) < (len(temp) - 1):
            subelement.tail = newline + indent * (level + 1)
        else:
            subelement.tail = newline + indent * level
        XML_Pretty(subelement, indent, newline, level=level + 1)


def XML_Pretty_All(filename: str):
    """
    :brief:             XML美化(整体美化)
    :param filename:    文件名
    :return:            None
    """
    tree = ET.parse(filename)
    root = tree.getroot()
    XML_Pretty(root)
    tree.write(filename)
