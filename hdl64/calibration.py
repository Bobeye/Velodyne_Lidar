import xml.etree.ElementTree
import pickle
import os


def getCals():

	if os.path.exists('CalibrationFile/cal.pkl'):
		with open('CalibrationFile/cal.pkl', 'rb') as p:
			cals = pickle.load(p)
			return cals
	else:

		root = xml.etree.ElementTree.parse('CalibrationFile/cal.xml').getroot()
		cals = []
		for subroot in root:
			for child in subroot:
				print (child.tag, child.attrib)
				if child.tag == 'points_':
					for subchild in child:
						for subsubchild in subchild:
							for pts in subsubchild:
								print (pts.tag, pts.text)
								if pts.tag == 'id_':
									cal = dict()
									cals += [cal]
								else:
									cals[-1][pts.tag] = float(pts.text)

		with open('CalibrationFile/cal.pkl', 'wb') as p:
			pickle.dump(cals, p)

		return cals



if __name__ == '__main__':
	cals = getCals()
	print (len(cals))
	for cal in cals:
		print (cal.keys())


	print (cals[4]['rotCorrection_'])

