import pathlib
from typing_extensions import final
from pylatex import Document, Section, Subsection, Tabular, Math, Figure, NoEscape, Command, MultiColumn, MultiRow, Center
from pylatex.utils import italic, bold
import pylatex.config as cf
import webbrowser

def generate_pdf(com, warning, final_constraints, t, steps):
    cf.active = cf.Version1(indent=False)
    filepath = pathlib.Path(__file__).parent
    imagespath = filepath.joinpath('images')
    reportpath = filepath.joinpath('report')

    geometry_options = {"tmargin": "2cm", "lmargin": "3cm"}
    doc = Document(geometry_options=geometry_options)

    doc.preamble.append(Command('title', 'Auto-generated Report for Softleg Jump'))
    doc.preamble.append(Command('author', 'Andrea Boscolo Camiletto e Marco Biasizzo'))
    doc.preamble.append(Command('date', NoEscape(r'\today')))
    doc.append(NoEscape(r'\maketitle'))

    with doc.create(Section('Settings of the problem')):
        doc.append('The problem to be solved is the jumping procedure of a SEA based leg.')
        doc.append('The time horizon is ' + str(t) + ' sec, with a number of steps of ' + str(steps) + '. That results in an open loop control at ' + str(round(steps/t,1)) +' Hz.')

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

    fconstraints, info = final_constraints
    with doc.create(Section('Final Constraints')):
        with doc.create(Center()) as centered:
            with doc.create(Tabular('c|c|c|c')) as table:
                table.add_hline()
                table.add_row((MultiColumn(4, align='|c|', data='Final Constraints Evaluation'),))
                table.add_hline()
                table.add_row(('Constraint', 'Lower Bound', 'End Result', 'Upper Bound'))
                table.add_hline()
                table.add_empty_row()
                for idx, constr in enumerate(fconstraints):
                    description = info[idx]
                    lenght = len(constr[0])
                    fixed_constr = [ x for x in constr]
                    for idx, single in enumerate(fixed_constr):
                        try:
                            iter(single)
                        except TypeError:
                            fixed_constr[idx] = [constr[idx]]
                    for counter, (low, val, up) in enumerate(zip(fixed_constr[0], fixed_constr[1], fixed_constr[2])):
                        try:
                            iter(val)
                            val = val[0]
                        except TypeError:
                            pass
                        if counter == 0:
                            table.add_row((MultiRow(lenght, data=description), round(low,4), round(val,4), round(up,4)))
                        else: 
                            table.add_row(('',  round(low,4), round(val,4), round(up,4)))
                    table.add_hline()          


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
