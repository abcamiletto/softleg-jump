import pathlib
from pylatex import Document, Section, Subsection, Tabular, Math, Figure, NoEscape, Command, MultiColumn, MultiRow
from pylatex.utils import italic, bold
import pylatex.config as cf
import webbrowser

def generate_pdf(com, warning):
    cf.active = cf.Version1(indent=False)
    filepath = pathlib.Path(__file__).parent
    imagespath = filepath.joinpath('images')
    reportpath = filepath.joinpath('report')
    space = NoEscape('\,\,')

    geometry_options = {"tmargin": "2cm", "lmargin": "3cm"}
    doc = Document(geometry_options=geometry_options)

    doc.preamble.append(Command('title', 'Auto-generated Report for Softleg Jump'))
    doc.preamble.append(Command('author', 'Andrea Boscolo Camiletto e Marco Biasizzo'))
    doc.preamble.append(Command('date', NoEscape(r'\today')))
    doc.append(NoEscape(r'\maketitle'))


    with doc.create(Section('Results')):
        doc.append('The vertical velocity of the leg, at the last timestep, is:')
        doc.append(Math(data=NoEscape(r'\boldsymbol{' + str(com['vel_y']) + '\,\, m/s}')))
        doc.append(NoEscape('Keep in mind that a good value may be between $0.5$ and $2.0 \, m/s$'))
    
    with doc.create(Section('Cost Function')):
        with doc.create(Figure(position='h!')) as joints:
            joints.add_image(str(imagespath.joinpath('cost.png')), width='250px')
            joints.add_caption('Cost Function Analysis')
        doc.append('')
        if warning:
            doc.append('WARN: There is more than an order of magnitude (200x) between the value of the cost function and the one of the final term. Keep an eye on that.')
        else:
            doc.append('The values of the cost function and the final term cost are decently balanced, well done.')
    doc.append(NoEscape(r'\pagebreak'))

    # with doc.create(Section('Final Constraints')):
    #     with doc.create(Tabular('|c|c|c|c|')) as table:
    #         table.add_hline()
    #         table.add_row()
    #         table.add_hline(1, 2)
    #         table.add_empty_row()
    #         table.add_row((4, 5, 6, 7))


# table3 = Tabular('|c|c|c|')
# table3.add_hline()
# table3.add_row((MultiColumn(2, align='|c|',
#                             data=MultiRow(2, data='multi-col-row')), 'X'))
# table3.add_row((MultiColumn(2, align='|c|', data=''), 'X'))
# table3.add_hline()
# table3.add_row(('X', 'X', 'X'))
# table3.add_hline()

    with doc.create(Section('Joints Behaviour')):
        with doc.create(Figure(position='h!')) as joints:
            joints.add_image(str(imagespath.joinpath('joints.png')), width='400px')
            joints.add_caption('Joints Dynamics')
        doc.append('On the figure above you can see on the rows the 3 joints and on the columns its position, velocity and acceleration.')
    doc.append(NoEscape(r'\pagebreak'))

    with doc.create(Section('Constraints')):
        with doc.create(Figure(position='h!')) as joints:
            joints.add_image(str(imagespath.joinpath('constr.png')), width='400px')
            joints.add_caption('Cost Function Analysis')
        doc.append('On the figure above you can see on the rows the different constraint, and on the column their plot per dimension. The titles are based on the configuration you set in the python file.')






    doc.generate_pdf(reportpath.joinpath('full'), clean_tex=False)
    webbrowser.open_new(str(reportpath.joinpath('full.pdf')))

if __name__ == '__main__':
    generate_pdf(dict(vel_y=1), True)
